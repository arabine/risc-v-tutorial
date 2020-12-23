#ifndef OST_COMMON_H
#define OST_COMMON_H

#define OST_ID_SPI_FOR_SDCARD   0

#define portDISABLE_INTERRUPTS()	__asm volatile( "csrc mstatus, 8" )
#define portENABLE_INTERRUPTS()		__asm volatile( "csrs mstatus, 8" )

#define portNOP() __asm volatile 	( " nop " )


#endif // OST_COMMON_H
