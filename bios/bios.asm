use16
org 0FE000h

start:
cld             ; Clear Direction Flag (DI will increment)
mov ax, 0AA55h    ; Value to fill memory with (e.g., 0xFF)
mov di, 0 ; Starting address of the buffer
mov cx, 10     ; Number of bytes to fill
rep stosw       ; Fill 100 bytes of Buffer with 0xFF


    ; Копирование строки hello в память по адресу 0xB0000
    mov ax, 0B000h      ; Сегмент видеопамяти (0xB0000)
    mov es, ax          ; Загружаем в ES
    mov di, 0           ; Смещение 0 (итого адрес 0xB0000)
    mov ax, 0F000h
    mov ds, ax
    mov si, 0E023h
    mov cx, 6          ; Длина строки (13 байт включая '$')
    cld                 ; Направление копирования вперед
    rep movsw           ; Копируем строку побайтово


    hlt

hello db 'H E L L O ', '$ '
; Заполняем пространство до адреса 0xFFFF0 нулями.
times (0xFFFF0 - $) db 90h

; Вектор сброса (Reset Vector) по адресу 0xFFFF0.
; Здесь мы вычисляем смещение метки 'start' относительно
; базового адреса 0F0000h. Результатом будет 0.
jmp far 0F000h:0E000h

; Заполняем оставшееся пространство
times (0x100000 - $) db 0