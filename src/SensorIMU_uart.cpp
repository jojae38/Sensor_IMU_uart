#include "SensorIMU_uart.h"

//to control gpio pin "sudo" is needed

// void init_uart(mraa::Uart &uart);
void control_gpio();

int main(int argc , char** argv)
{
    ros::init(argc, argv, "Sensor_IMU_uart_node");
    SensorIMU_uart uart;
    ros::spin();
    // control_gpio();
    // mraa::Uart uart(0);
    // init_uart(uart);
    // while(true)
    // {
    //     control_gpio();
    // }
    return 0;
}
void control_gpio()
{
    mraa::Gpio gpio(7);//gpio pin
    gpio.dir(mraa::Dir::DIR_OUT);//output or input 
    gpio.write(1);//write 1 -> 3.3V
}

// MRAA no.	Function	Rpi_GPIO	Sysfs_GPIO	mraa_device
// 1	    3V3 VCC			
// 2	    5V VCC			
// 3	    I2C_SDA	    2	        462	        I2C0
// 4	    5V VCC			
// 5	    I2C_SCL	    3	        463	        I2C0
// 6	    GND			
// 7	    GPIO(4)	    4	        433	
// 8	    UART_TX	    14	        477	        UART0
// 9	    GND			
// 10	    UART_RX	    15	        476	        UART0
// 11	    UART_RTS	17	        478	        UART0
// 12	    I2S_CLK	    18	        326	
// 13	    GPIO(27)	27	        432	
// 14	    GND			
// 15	    GPIO(22)	22	        431	
// 16	    PWM3	    23	        471	        PWM3
// 17	    3V3 VCC			
// 18	    GPIO(24)	24	        405	
// 19	    SPI0_MOSI	10	        422	        SPI0
// 20	    GND			
// 21	    SPI0_MISO	9	        421	        SPI0
// 22	    GPIO(25)	25	        402	
// 23	    SPI0_SCL	11	        418	        SPI0
// 24	    SPI0_CS0	8	        419	        SPI0
// 25	    GND			
// 26	    SPI0_CS1	7	        420	        SPI0
// 27	    ID_SD	    0	        464	        I2C1
// 28	    ID_SC	    1	        465	        I2C1
// 29	    GPIO(5)	    5	        430	
// 30	    GND			
// 31	    GPIO(6)	    6	        404	
// 32	    PWM0	    12	        468	        PWM0
// 33	    PWM1	    13	        469	        PWM1
// 34	    GND			
// 35       I2S_FRM	    19	        327	
// 36	    UART_CTS	16	        479	        UART0
// 37	    GPIO(26)	26	        403	
// 38	    I2S_DIN	    20	        328	
// 39	    GND			    
// 40	    I2S_DOUT	21	        329	