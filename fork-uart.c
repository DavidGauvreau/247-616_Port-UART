/**
 * @file    fork-uart.c
 * 
 * @brief Serial Port Programming in C (Serial Port Read / write with fork)  
 * Non Cannonical mode
 * Sellecting the Serial port Number on Linux   
 * /dev/ttyUSBx - when using USB to Serial Converter, where x can be 0,1,2...etc 
 * /dev/ttySx   - for PC hardware based Serial ports, where x can be 0,1,2...etc  
 * termios structure -  /usr/include/asm-generic/termbits.h  
 * use "man termios" to get more info about  termios structure
 * @author  Kevin Cotton
 * @author  David gauvreau
 * @date    2025-10-28
 */	
#define _GNU_SOURCE

#include <stdio.h>
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions 
#include <unistd.h>  // UNIX Standard Definitions
#include <errno.h>   // ERROR Number Definitions
#include <sys/wait.h>
#include <string.h>

//Déclarations de foncitons privées
void InitSerialPortParent(void);
void InitSerialPortEnfant(void);
void ReadSerialData(void);
void WriteSerialData(void);


// device port série à utiliser 
//const char *portTTY = "/dev/ttyGS0";
//const char *portTTY = "/dev/ttyS0";
//const char *portTTY = "/dev/ttyS1";
//const char *portTTY = "/dev/ttyS2";
//const char *portTTY = "/dev/ttyS3";
//const char *portTTY = "/dev/ttyS4";
//const char *portTTY = "/dev/ttyS5";
const char *portTTY = "/dev/ttyUSB0"; // ttyUSB0 is the FT232 based USB2SERIAL Converter

int fd_SerialPort; // File Descriptor
pid_t pid; //process id
int flag_ExitProcess = 0;

char buffer[256];

void main(void)
{
    pid = fork();

    if (pid == -1) 
    {
        perror("fork");
        return;
    }

    if (pid == 0)
    {
        printf("\nJe suis le processus Fils, j'écris sur le port série ce que j'entends sur la console (terminal)...\n");
        
        InitSerialPortEnfant();
        
        while (1)
        {
            WriteSerialData();

            if (flag_ExitProcess == 1)
            {
                printf("\nFin du processus Fils\n");
                break;
            }
        }

        close(fd_SerialPort); // Close the serial port
    }
    else
    {
        printf("\nJe suis le processus Père, j'écris sur la console (terminal) ce que j'entends sur le port série...\n");

        InitSerialPortParent();
        
        
        while (1)
        {
            ReadSerialData();

            if (flag_ExitProcess == 1)
            {
                printf("\nFin du processus Père\n");
                break;
            }
        }

        close(fd_SerialPort); // Close the serial port
        
        int status;
        waitpid(pid, &status, 0);  // Attend la fin de l’enfant
    }
    
    
}

void InitSerialPortEnfant(void)
{
	// Opening the Serial Port 
	fd_SerialPort = open(portTTY, O_RDWR | O_NOCTTY | O_NDELAY);
								// O_RDWR Read/Write access to serial port           
								// O_NOCTTY - No terminal will control the process   
								// O_NDELAY -Non Blocking Mode,Does not care about-  
								// -the status of DCD line, Open() returns immediatly                                        
	if(fd_SerialPort == -1) // Error Checking
    {
		printf("\n Erreur! Enfant ouverture de %s \n", portTTY);
    }
	else
    {
		//printf("\n Enfant Ouverture de %s reussit ", portTTY);
    }

	// Setting the Attributes of the serial port using termios structure 
	struct termios SerialPortSettings;	// Create the structure 
	tcgetattr(fd_SerialPort, &SerialPortSettings);	// Get the current attributes of the Serial port
	// Setting the Baud rate
	cfsetispeed(&SerialPortSettings, B115200); // Set Read  Speed   
	cfsetospeed(&SerialPortSettings, B115200); // Set Write Speed  
	// 8N1 Mode 
	SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity
	SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
	SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size
	SerialPortSettings.c_cflag |=  CS8;      //Set the data bits = 8
	SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver, Ignore Modem Control lines 

	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);	// Disable XON/XOFF flow control both i/p and o/p

	SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode, Disable echo, Disable signal  

	SerialPortSettings.c_oflag &= ~OPOST;	//No Output Processing

	if((tcsetattr(fd_SerialPort, TCSANOW, &SerialPortSettings)) != 0) // Set the attributes to the termios structure
    {
		printf("\n  Erreur! Enfant configuration des attributs du port serie\n");
    }
}

void InitSerialPortParent(void)
{
    // Opening the Serial Port 
    // O_RDWR   - Read/Write access to serial port 
    // O_NOCTTY - No terminal will control the process
    // Open in blocking mode,read will wait 
	fd_SerialPort = open(portTTY, O_RDWR | O_NOCTTY);  
							
	if(fd_SerialPort == -1) // Error Checking
    {
		printf("\n Erreur! Parent ouverture de %s \n", portTTY);
    }
	else
    {
		//printf("\n Parent Ouverture de %s reussit ", portTTY);
    }

    // Setting the Attributes of the serial port using termios structure 
	struct termios SerialPortSettings;	// Create the structure 
	tcgetattr(fd_SerialPort, &SerialPortSettings);	// Get the current attributes of the Serial port 
	// Setting the Baud rate
	cfsetispeed(&SerialPortSettings, B115200); // Set Read Speed  
	cfsetospeed(&SerialPortSettings, B115200); // Set Write Speed  
	// 8N1 Mode 
	SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity 
	SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
	SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size 
	SerialPortSettings.c_cflag |=  CS8;      // Set the data bits = 8  
	SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver, Ignore Modem Control lines
	
	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both i/p and o/p

	SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode, Disable echo, Disable signal  
    //SerialPortSettings.c_lflag |= ICANON; // enable canonical mode

	SerialPortSettings.c_oflag &= ~OPOST;	// No Output Processing

	// Setting Time outs 
	SerialPortSettings.c_cc[VMIN] = 1; // Read at least X character(s) 
	SerialPortSettings.c_cc[VTIME] = 0; // Wait Xsec between each character (0 for indefinetly) 

	if((tcsetattr(fd_SerialPort, TCSANOW, &SerialPortSettings)) != 0) // Set the attributes to the termios structure
    {
		printf("\n  Erreur! Parent configuration des attributs du port serie\n");
    }

    tcflush(fd_SerialPort, TCIFLUSH);  // Discards old data in the rx buffer
}

void ReadSerialData(void)
{
    // Read data from serial port 
	char read_buffer[32];   // Buffer to store the data received 
	int  bytes_read = 0;    // Number of bytes read by the read() system call 
	int i = 0;

	bytes_read = read(fd_SerialPort, &read_buffer, 32); // Read the data 
		
	printf("\nProcessus père : nombres d'octets reçus : %d --> ", bytes_read); // Print the number of bytes read

	for(i=0; i<bytes_read; i++)	 // printing only the received characters
    {
		printf("%c", read_buffer[i]);

        if (read_buffer[i] == '!')
        {
            flag_ExitProcess = 1;
            break;
        }
    }

	printf("\n");
}

void WriteSerialData(void)
{
    //printf("\nEntrez un texte à envoyer : \n");
    if (fgets(buffer, sizeof(buffer), stdin) != NULL) 
    {
        // fgets lit aussi le '\n' final, on le remplace par un \0
        buffer[strcspn(buffer, "\n")] = '\0';
    }

    size_t len = strlen(buffer);// Donne la taille du texte de l'utilisateur
    int bytes_written = write(fd_SerialPort, buffer, len); // use write() to send data to port 

    for (int i = 0; i < len; i++)
    {
        if (buffer[i] == 'q')
        {
            flag_ExitProcess = 1;
            break;
        }
    }
}