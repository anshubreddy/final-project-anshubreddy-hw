#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

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
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("tcsetattr");
        exit(EXIT_FAILURE);
    }
}

int main()
{
    const char *portname = "/dev/ttyUSB0";
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0)
    {
        perror("open");
        return EXIT_FAILURE;
    }

    setup_serial(fd);
    char buf[256];
    char message[256] = {0};
    int msg_index = 0;

    while (true)
    {
        int n = read(fd, buf, sizeof(buf) - 1);
        if (n > 0)
        {
            buf[n] = '\0';

            // Append incoming data to the message buffer
            strncat(message, buf, n);
            msg_index += n;

            // Process complete messages ending with '\n'
            char *newline_pos = strchr(message, '\n');
            while (newline_pos != NULL)
            {
                // Extract the complete message
                size_t message_len = newline_pos - message + 1;
                char complete_message[256];
                strncpy(complete_message, message, message_len);
                complete_message[message_len] = '\0';

                // Print the formatted sensor data
                printf("%s", complete_message);

                // Shift remaining data in the buffer
                memmove(message, newline_pos + 1, msg_index - message_len);
                msg_index -= message_len;

                // Check for another complete message
                newline_pos = strchr(message, '\n');
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
