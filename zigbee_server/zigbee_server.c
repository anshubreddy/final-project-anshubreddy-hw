#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

//
// Circular buffer definitions
//
#define BUFFER_SIZE 1024
char circular_buffer[BUFFER_SIZE];
int head = 0;      // Index to write new data
int tail = 0;      // Index to read data from
int data_size = 0; // Number of bytes currently in the buffer

// Function to configure the serial port
static void setup_serial(int fd)
{
   struct termios tty;
   if (tcgetattr(fd, &tty) != 0)
   {
      perror("tcgetattr");
      exit(EXIT_FAILURE);
   }

   cfsetospeed(&tty, B9600);
   cfsetispeed(&tty, B9600);
   tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit data
   tty.c_iflag &= ~IGNBRK;                      // Disable break processing
   tty.c_lflag = 0;                             // No canonical mode or echo
   tty.c_oflag = 0;                             // No output processing
   tty.c_cc[VMIN] = 1;                          // Read at least 1 byte
   tty.c_cc[VTIME] = 5;                         // 0.5-second timeout
   tty.c_iflag &= ~(IXON | IXOFF | IXANY);      // No software flow control
   tty.c_cflag |= (CLOCAL | CREAD);             // Enable receiver, local mode
   tty.c_cflag &= ~(PARENB | PARODD);           // No parity
   tty.c_cflag &= ~CSTOPB;                      // 1 stop bit
   tty.c_cflag &= ~CRTSCTS;                     // No hardware flow control

   if (tcsetattr(fd, TCSANOW, &tty) != 0)
   {
      perror("tcsetattr");
      exit(EXIT_FAILURE);
   }
}

int main()
{
   const char *portname = "/dev/ttyUSB0"; // Serial port for XBee Router
   int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);

   if (fd < 0)
   {
      perror("open");
      return EXIT_FAILURE;
   }

   setup_serial(fd);
   char buf[256];  // Temporary buffer for reading from serial port

   while (true)
   {
      //
      // Read data from the serial port
      //
      int n = read(fd, buf, sizeof(buf));
      if (n > 0)
      {
         //
         // Write data to the circular buffer
         //
         for (int i = 0; i < n; i++)
         {
            circular_buffer[head] = buf[i];
            head = (head + 1) % BUFFER_SIZE;
            if (data_size < BUFFER_SIZE)
            {
               data_size++;
            }
            else
            {
               //
               // Buffer full: overwrite oldest data
               //
               tail = (tail + 1) % BUFFER_SIZE;
            }
         }

         //
         // Process complete messages from the circular buffer
         //
         while (true)
         {
            //
            // Look for a newline to identify a complete message
            //
            int i;
            for (i = 0; i < data_size; i++)
            {
               int index = (tail + i) % BUFFER_SIZE;
               if (circular_buffer[index] == '\n')
               {
                  break;
               }
            }

            if (i == data_size)
            {
               //
               // No complete message found yet
               //
               break;
            }

            //
            // Extract the message up to the newline
            //
            char message[256]; // Buffer for a single message
            int msg_len = 0;
            while (msg_len < sizeof(message) - 1 && data_size > 0)
            {
               char c = circular_buffer[tail];
               tail = (tail + 1) % BUFFER_SIZE;
               data_size--;
               message[msg_len++] = c;

               if (c == '\n')
               {
                  break;
               }
            }

            message[msg_len] = '\0'; // Null-terminate the message

            //
            // Print the message to the server monitor
            //
            printf("Received: %s", message);
            fflush(stdout);  // Ensure output is displayed immediately
         }
      }
      else if (n < 0)
      {
         perror("read");
         break;
      }
   }

   close(fd);
   return 0;
}
