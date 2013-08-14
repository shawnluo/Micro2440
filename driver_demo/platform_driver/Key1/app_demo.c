3.Ӧ�ò���Դ���

#include <stdio.h>

#include <stdlib.h>

#include <unistd.h>

#include <sys/ioctl.h>

#include <sys/types.h>

#include <sys/stat.h>

#include <fcntl.h>

#include <sys/select.h>

#include <sys/time.h>

#include <errno.h>

int main(void)

{

int buttons_fd;

char buttons[2] = {'0', '0'};

buttons_fd = open("/dev/s3c2410-buttons", 0);

if (buttons_fd < 0) {

perror("open device buttons");

exit(1);

}

while(1)
{

char current_buttons[2];

int count_of_changed_key;

int i;

if (read(buttons_fd, current_buttons, sizeof current_buttons) != sizeof current_buttons) {

perror("read buttons:");

exit(1);

}

for (i = 0, count_of_changed_key = 0; i < sizeof buttons / sizeof buttons[0]; i++) {

if (buttons[i] != current_buttons[i]) {

printf("%skey %d is %s", count_of_changed_key? ", ": "", i+1, buttons[i] == '0' ? "up" : "down");

count_of_changed_key++;

}

}

if (count_of_changed_key) {

printf("\n");

}

}

close(buttons_fd);

return 0;

}

 
/*
ƽ̨�豸���ԣ�

�ڳ����ն��£�

cd /home/platform/device

insmod device.ko

cd ../driver

insmod driver.ko

cd ../

./buttons

Ȼ��ͨ��Mini2440������İ������۲쵽�����ն˵İ�����Ϣ��

 

�豸��������ԣ�

�ڳ����ն��£�

cd /sys/platform/ s3c2410-buttons

ls�󣬻���ʾbuttons��һĿ¼

��ȡ�豸���ԣ�cat buttons

�޸��豸���ԣ�echo modify>buttons
*/