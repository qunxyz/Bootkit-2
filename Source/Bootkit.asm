HOME_SEG	EQU	0x7000;
MODULE_SIZE	EQU	0x8000;
DEST_INT	EQU 0x13;		;destination int 

%define BIOS_BOOT
;%define FD_BOOT

;--------------------------------------------------------
;					这里是ISA头部
;--------------------------------------------------------	
	[bits 16]
	ORG 0
ROM_START:
	;=====================================================
OPT_ROM_HEADER:				;Option ROM Header
	ROM_SIG:					;3 Bytes
	
		%ifdef BIOS_BOOT
			_MOD_SIG	dw	0xAA55
			_MOD_LEN	db	(MODULE_SIZE/512)
		%endif
		
		%ifdef FD_BOOT
			jmp	CODE_START			;暂时使用jmp addr代替
		%endif
		
	ROM_ENTRY:					;4 Bytes
			jmp CODE_START			;BIOS Far Call this addr
			db	0					;ret instructure
	ROM_RSVD:					;19 Bytes
			dd	0
			dd	0
			dd	0
			dd	0
			dw	0
			db	0
	EXPANSION_HEADER:
		dw	0					;2 Bytes
;========================================================================================
;	after moved to revd mem,the 'Option ROM Header' is treated as 'GLOBAL_DATA_BLOCK'
;========================================================================================
;	;>>>记录代码段
;	CODE_SEG	equ		(	-	ROM_START)	;offset 0	dw	0	signature
;	;记录中断请求功能号
;	ORIG_FUNC	equ		2							;offset	2	db	0	length
;	;记录原始int_isr地址
;	ORIG_ISR	equ		(ROM_ENTRY	-	ROM_START)	;offset	3	dd	0	jmp near addr  &&  ret
;	;>>>记录32位代码开始地址
;	ENTRY_32	equ		(ROM_RSVD	-	ROM_START)	;offset	7	dd	0	rsvd[0]
;	;>>>记录Driver.SYS开始位置
;	DRV_START	equ		(ENTRY_32	+	4		 )	;offset 11	dd	0	rsvd[1]
;	;Driver.sys大小
;	DRV_SIZE	equ		(ENTRY_32	+	16		 )	;offset	23	dw	0	
;	;供SUB32使用
;	SCREEN_XY	equ		(ENTRY_32	+	8		 )	;offset	15	dd	0	rsvd[2]
;	;
;	UNUSED1		equ		(ENTRY_32	+	12		 )	;offset	19	dd	0	rsvd[3]
;	;驱动入口(HookFunc)物理地址
;	DRIVER_ENTRY_POINT	equ	(ENTRY_32	+	18	 )	;call DRIVER_32
;	;call dword [Driver_entry]	指令地址+2
;	;P_INSTR_DRV_SUB		equ	(ENTRY_32	+	22	 )	;call dword [xxxxxxxx]
;	SECTOR_CNT	equ		(ENTRY_32	+	22		 )  ;save int 0x13's param
	ENTRY_32	equ		ROM_START+0		;	dd
	CODE_SEG	equ		ROM_START+4		;	dw;--------->
	ORIG_FUNC	equ		ROM_START+6		;	db;
	SECTOR_CNT	equ		ROM_START+7		;	db;--------->
	DRV_START	equ		ROM_START+8		;	dd
	DRV_SIZE	equ		ROM_START+12	;	dw
	;--------------------------------------------------->
	ORIG_ISR	equ		ROM_START+16	;	dd
	DATA_BUF	equ		ROM_START+20	;	? dup db 0
	ORIG_ISR_19H	equ		ROM_START+20	;dd
	
	ORIG_ES		equ		ROM_START+24		;dw
	ORIG_BX		equ		ROM_START+26		;dw
	CODE_FLAG	equ		ROM_START+28
	
	
	
	;下面的代码只执行一次，之后当作数据块来使用
							;	dd x 4	0
							;	dw	0
							;	db	0
							;	dw	0
;========================================================================================
	
	
	
	
	
	
;**********************************************************************
CODE_START:
	;------------------------------------------------------
	;		Read Rest Data[debug Utils]
	;------------------------------------------------------
%ifdef FD_BOOT
.ReadRest_Data:

	dbg_data_seg	equ	0x7E0
	mov bx,0x7e0
	mov es,bx
	xor bx,bx				;es:bx=0x7E00
	mov ax,0x0211			;al=0x11(8.5KB),ah=0x2	(0x12 x 512 =9KB)
	xor dx,dx				;dh=0;dl=0	;floppy head:0
	mov cx,0x0002			;sector:2
	int 0x13
.ReadRest_End:	
%endif

%ifdef RELEASE_ISA_ROM
	mov al,_MOD_LEN
	xor bl,bl
	;mov 
%endif
	;------------------------------------------------------
	;		rsvd mem for my self
	;------------------------------------------------------
.RsvdMem_start:
	xor ax,ax
	mov ds,ax
	mov es,ax
	mov ax,word[ds:0x0413]		;ax=BaseAddr (KB)

	sub ax,(MODULE_SIZE/1024)	;Save MODULE_SIZE Byte HighMem For Me 
	mov word [ds:0x0413],ax
	shl ax,6					;(BaseAddr<<10)>>4
	mov es,ax					;es=dest_segment
.RsvdMem_end:

	;------------------------------------------------------
	;	move myself to rsvd mem and jmp dest_seg:offset 0
	;------------------------------------------------------
MoveCode_start:
	call .MoveCode				;Get ip
.MoveCode:

	IP_OFFSET		equ (.MoveCode-ROM_START)
	
%ifdef BIOS_BOOT
	MOD_LEN_OFFSET	equ (.MoveCode-_MOD_LEN)
%endif

	pop 	si					;si=.ModeCode
	
%ifdef BIOS_BOOT
	;
	;	Fix ISA Module's Module_Length && ISA_MODULE_CRC
	;
	push	si
	mov		di,si				;save si
	sub		si,MOD_LEN_OFFSET
	mov 	al,byte [cs:si]		;get value	of mod len
	mov		byte [cs:si],0		;set module len == 0,this will cause a release mem of isa module after isa module return...
	add		di,CRC_OFFSET
	mov		ah,byte [cs:di]		;get module crc
	add		ah,al
	mov		byte [cs:di],ah		;set new crc of module...
	pop 	si
%endif

	xor 	di,di				;destination=ES:0
	mov 	ax,cs				;get source segment
	mov 	ds,ax
	sub 	si,IP_OFFSET		;copy from code_start
	mov 	cx,MODULE_SIZE
	cld
	rep 	movsb				;[ds:si++]=[es:di++]

	push 	es					;generate return stack
	push 	REAL_CODE_OFFSET
	retf
MoveCode_end:








;**********************************************************************
	;------------------------------------------------------------
	;						Real Code Body						;
	;------------------------------------------------------------
REAL_CODE_START:
	REAL_CODE_OFFSET	equ		(REAL_CODE_START-ROM_START)
	;save env
	;hook int 0x13
	;Load MBR
	;jmp $
.save_env:
	;addr currently should be [xxxx:0]
	;jmp $
	
	xor eax,eax
	mov ax,cs
	mov ds,ax
	mov es,ax
	
	mov word [cs:CODE_SEG],ax				;save code seg val
	shl eax,4
	mov edx,eax;
	add eax,(DRIVER_START-ROM_START)		;
	mov dword [cs:DRV_START],eax			;save binary start_addr	

;这个数据暂时不需要
;	mov ax,_DRIVER_SIZE
;	mov word [cs:DRV_SIZE],ax 				;save driver size

;	mov eax,edx
;	add eax,(DRIVER_ENTRY-ROM_START)		;code_base + DRIVER_ENTRY
;	mov dword [cs:DRIVER_ENTRY_POINT],eax	;save driver_entry_point in addr
;											;[DRIVER_ENTRY_POINT]=code_base + DRIVER_ENTRY
;	
;	mov eax,edx
;	mov dword [cs:INSTR_GLOBAL],eax			;save pGlobal in block
;											;[V_GLOBAL]=code_base
;											;push 
;											;V_GLOBAL:
;											;				dd	??
;											;
;	add eax,(INSTR_DRIVER_32-ROM_START)
;	mov dword [cs:P_INSTR_DRV_SUB],eax		;save call [xxxx] instr addr
;											;[V_SUB_ENTRY]=code_base + V_SUB_ENTRY
;											;call dword [V_SUB_ENTRY]
;											;call dword
;											;V_SUB_ENTRY:
;											;				dd	??
;											;
;	
;	;sub32任务：
;	;	+-->修改驱动的EntryPoint为DRIVER_ENTRY_POINT域值
;	;	+-->修改压入的参数
;	;	+-->修改call dword [xxxx]指向自己的函数
;	;	+-->在自己的函数入口中获得压入的参数，获取KeAPI，展开驱动，启动驱动


;--------------------------------------------------------
%ifdef FD_BOOT
.load_mbr_0:

		xor ax,ax
		mov es,ax	;es=0x0
		mov ax,0x0201;
		mov dx,0x0080;
		mov cx,1;
		mov bx,0x7c00;
		int 0x13;		;==>es:bx
		;jmp later,don't hook myself...
%endif	
;--------------------------------------------------------

%ifdef FD_BOOT
.install_isr_13h:
	xor ax,ax
	mov es,ax
	
	mov eax,dword [es:(DEST_INT *4)]
	mov dword [cs:ORIG_ISR],eax

	mov word [es:(DEST_INT *4)],(ISR_13H_START-ROM_START)
	mov word [es:(DEST_INT *4 + 2)],cs
%endif

;--------------------------------------------------------



%ifdef BIOS_BOOT
.install_isr_19h:
	xor ax,ax
	mov es,ax
	
	mov eax,dword [es:0x64]	
	mov dword [cs:ORIG_ISR_19H],eax

	mov word [es:0x64],(ISR_19H_START-ROM_START)
	mov word [es:0x66],cs	
%endif
;--------------------------------------------------------	
	;计数清0
	;mov word [cs:CODE_FLAG],0;

;	;===============================================
;	mov ax,0x1234
;	call hex_dump2
;	;______________________________________
;	mov ah,3h ;换行
;	mov bh,0
;	int 10h
;	inc dh
;	mov dl,0
;	mov ah,2h
;	mov bh,0
;	int 10h
;	;______________________________________
;	mov ax,0x1234
;	call hex_dump2
;	
;	jmp $
;	
;	;===============================================
%ifdef FD_BOOT
	jmp word 0x0:0x7c00	;leave control
	;jmp $				;never here		
%endif
;--------------------------------------------------------

%ifdef BIOS_BOOT
	xor ax,ax			;return 0,module init okay...?
	retf
%endif
;--------------------------------------------------------




;**********************************************************************
;=============================================================
;					19h中断服务例程							 ;
;=============================================================
ISR_19H_START:

%ifdef BIOS_BOOT
	pushf
	pusha
	
	xor ax,ax
	mov es,ax	
	;recover 19h,this seems tobe unnessary...
	mov eax,dword [cs:ORIG_ISR_19H]
	mov dword [es:0x64],eax
	;hook dest_int
	mov eax,dword [es:(DEST_INT *4)]
	mov dword [cs:ORIG_ISR],eax
	
	mov word [es:(DEST_INT *4)],(ISR_13H_START-ROM_START)
	mov word [es:(DEST_INT *4 + 2)],cs

	popa
	popf
	
	jmp far word [cs:ORIG_ISR_19H]			;redirect to orig isr..
	
	;never here...
	jmp $
%endif


;**********************************************************************
;=============================================================
;					13h中断服务例程							 ;
;=============================================================
ISR_13H_START:
;
;faint 刚进入中断时,！！！DS未知！！！！！，cs确定
;here!!!!exhaust a lot of time!!!
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;jmp far word [cs:(ORIG_ISR-.CODE)]
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	
	
	pushf			;save flags for cmp instr
	cmp ah,0x2
	je .isr_do_hook
	cmp ah,0x42
	je .isr_do_hook
.isr_do_int:
	popf
	jmp far word [cs:ORIG_ISR];
	;never here
.isr_do_hook:
	popf							;banlance
	mov byte [cs:ORIG_FUNC],ah
	mov byte [cs:SECTOR_CNT],al
	
	;jmp far word [cs:(ORIG_ISR-.CODE)]
	
	;-----------------emulate int call's stack farme-----------------
	pushf			;push caller's flags and call to generate a int call's stack frame
	call far word [cs:ORIG_ISR]	;stack >>>[flags][cs][ip]|[flags][cs][ip]>>>
;	;----------------------------------------------------------------
;	;after this stack is balance,ie.>>>[flags][cs][ip]>>>
;	;----------------------------------------------------------------
	;==================do filter here================================
.isr_save_env:
	pushf
	push es
	push ds
	pusha
	
;	cmp bx,word [cs:ORIG_BX]
;	je .okay_bx
;	jmp $
;.okay_bx
;	mov bx,word [cs:ORIG_ES]
;	mov es,bx
;	mov bx,word [cs:ORIG_BX]
;	;=====================================
;	xor eax,eax
;	mov ax,es
;	shl eax,4
;	and ebx,0xffff
;	add	eax,ebx
;	mov ebx,eax
;	and ebx,0x0000000F		;set bx=eax&0xf
;	shr eax,4
;	mov es,ax
;	
;	
;	;=====================================

	mov al,byte [cs:SECTOR_CNT]		;restore al,VMWare need,but bochs works well without this command...???
	cmp byte [cs:ORIG_FUNC],0x42
	jne .isr_func_2h
.isr_func_42h:
	;入口参数如下：
	;AH = 42(读子功能)
	;DL = 驱动器号，80/81表示硬盘
	;DS:SI 指向一个10H的磁盘地址包
	;磁盘地址包的数据结构如下：
	;第0字节：包长度(固定为10h)
	;第1字节：保留，必须为0
	;第2、3字节：所读扇区数
	;第4～5字节：内存目标缓冲区首址偏移
	;第6～7字节：内存目标缓冲区首址段址
	;第8～15字节：扇区LBA号码
	;struct read_info__
	;{
	;	UCHAR Len;
	;	UCHAR Rsvd0;
	;	USHORT SectCnt;
	;	USHORT Buf_Offset;
	;	USHORT Buf_Segment;
	;	LONGLONG LBA;????
	;};
	
	;ds:si--->Param Block
	
	mov ax,word [ds:(si+2)];
	mov bx,word [ds:(si+6)];
	mov es,bx;							;fix es
	mov bx,word [ds:(si+4)];			;fix bx
	
.isr_func_2h:
	;es:bx-->buffer
	;al=sect_count	//0x42->ax (ah=0?)

	xor cx,cx
	mov cl,al
	shl cx,9		;sect_cnt*512
	sub cx,11		;SIG_CODE_LEN
	
.mem_find_start:	
	cmp dword [es:bx],0xABF3C033
	jne .next_data_0
	cmp dword [es:(bx+4)],0x0FD8200F
	jne .next_data_0
	mov eax,dword [es:(bx+8)]
	and eax,0x00FFFFFF
	cmp eax,0x00A1D822
	jne .next_data_0
	;okay here
	;jmp $
	;jmp .uninstall_isr;
.isr_install_entry32:
	CALL_I32_INSTR 	equ 0x15FF		;CALL r/m32 FF /2 子程序调用(32位间接寻址)
									;call dword [0xffeeddcc]  <====>  0xFF 0x15 0xCC 0xDD 0xEE 0xFF 
									;call 0x11223344          <====>  0xE8 0x3F 0x33 0x22 0x11
	mov word [es:bx],CALL_I32_INSTR
	xor eax,eax
	mov ax,cs
	shl eax,4						;eax=code_base
	add eax,ENTRY_32				;	
	mov dword [es:(bx+2)],eax		;call dword [ENTRY_32]
	

	sub eax,ENTRY_32
	add eax,(SUB32_START-ROM_START)
	mov dword [cs:ENTRY_32],eax		;[ENTRY_32]=EntryPoint32

.uninstall_isr:
	xor ax,ax
	mov es,ax
	mov eax,dword [cs:ORIG_ISR]
	mov dword [es:(DEST_INT *4)],eax
	
	jmp .isr_restore_env			;
	
.next_data_0:
	inc bx
	;dec cx!!!!!!!!!!!!!!!!!!!!!!!!!
	loopnz .mem_find_start	

.isr_restore_env:

	popa
	pop ds
	pop es
	popf
	
.isr_13h_ret:
	;================================================================
	retf 2			;skip user's flags
.isr_13h_end
;================================================================






;**********************************************************************
;=====================================================
;input ds:si=string
;=====================================================
print_s:
	lodsb				;[ds:si]--->al
	cmp al,0x0
	je .print_s_end
	mov ah,0xe			;bios call :print al
	mov bx,0x7
	int 0x10
	jmp print_s
.print_s_end
	ret

	
;=====================================================
;input ax=data to be show
;show word in hex
;=====================================================
hex_dump2:
	push ax
	shr ax,8
	call hex_dump
	pop ax
	push ax
	call hex_dump
	pop ax
	ret
.hex_dump_end

;=====================================================
;input al=data to be show
;show byte in hex
;=====================================================
hex_dump:
	push ax
	shr	ax,4
	call hex_dump_byte_low			;show high 4 bit of al
	pop ax
	push ax
	call hex_dump_byte_low			;show low 4 byte of al
	pop ax
	ret
.hex_dump_end
	
;=====================================================
;changed register:ax,bx
;this function show al's low 4 bit (in hex)
;=====================================================
hex_dump_byte_low:
	;input =al;
	push bx
	push ax

	mov bl,'0'
	and al,0xF
	cmp al,10
	jl	.less_10
	mov bl,'A'
	sub al,10
.less_10
	add al,bl
	;-----------------
	mov ah,0xe
	mov bx,7
	int 0x10
	;-----------------
	pop ax
	pop bx
	ret
.hex_dump_byte_end
;================================================================
;[bits 32]
;Phase_2:
;BlSetupForNt(BlLoaderBlock)
;    +-->call [addr]-----+
;                        |
;                        V
;EntryPoint32:
;.Entry32_start:
;	push ebp
;	mov ebp,esp
;	;---------------
;	;call sub32 first ;;; then do_code
;	;save all
;	pushf
;	pushad
;		;jmp $
;		;mov ax,word [ds]
;	popad
;	popf
;	
;	;--------------------------------------
;	mov eax,dword [ebp+4]	;get ret_addr
;	add eax,4				;set ret_addr-->mov eax,dword [xxxx],after code mov cr3,eax
;	mov dword [ebp+4],eax	;fix stack
;
;	xor eax,eax
;	rep stosd
;	mov eax,cr3
;	mov cr3,eax
;	
;	mov esp,ebp
;	pop ebp
;	ret
;.Entry32_end:
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;DriverEntryPoint HookFunc
;DRIVER_ENTRY:
;	push ebp
;	mov ebp,esp
;	;--------------------------------
;	push dword [ebp+8]	;pDriverObject
;	;--------------------------------
;	;push 0x12345678
;	db	0x68
;INSTR_GLOBAL:
;	dd	0x0				;pGlobalData
;	;--------------------------------
;	dw	CALL_I32_INSTR
;INSTR_DRIVER_32:
;	dd	0xaabbccdd		;Driver
;	;--------------------------------
;	leave
;	ret 4
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	


REAL_CODE_END:

DATA_EXT_START:
	times 510-($-$$) 	db 0
DATA_EXT_END:

	dw 0xaa55







;**********************************************************************
;================================================================
;	here is 32bit utils ;building env for drivers 
;================================================================
SUB32_START:

	incbin './SUB32.BIN'
	
SUB32_END:



;**********************************************************************
;================================================================
;	here is builtin driver,it seems ...
;================================================================
DRIVER_START:

	incbin './Baks.sys'
	
DRIVER_END:

_DRIVER_SIZE	equ		(DRIVER_END-DRIVER_START)



;**********************************************************************
;================================================================
;	end...
;================================================================
END_OF_SECTION:
.align_sector_0

%ifdef FD_BOOT
	times ((($-$$)/512)+1)*512-($-$$)-1 db 0
%endif

%ifdef BIOS_BOOT
	times (MODULE_SIZE-($-$$)-1)	db		0
%endif

	MODULE_CKSUM					db		0
	
	
	CRC_OFFSET		equ 			(MODULE_CKSUM-MoveCode_start.MoveCode)