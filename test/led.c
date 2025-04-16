// #include <stdlib.h>
// #include <stdio.h>
// #include <string.h>
// #include <unistd.h>
// #include <fcntl.h> //define O_WRONLY and O_RDONLY

// // LED 引脚
// #define SYSFS_GPIO_EXPORT "/sys/class/gpio/export"
// #define SYSFS_GPIO_UNEXPORT "/sys/class/gpio/unexport"
// #define SYSFS_GPIO_RST_PIN_VAL "440"  // 根据实际情况修改
// #define SYSFS_GPIO_RST_DIR "/sys/class/gpio/gpio440/direction"
// #define SYSFS_GPIO_RST_DIR_VAL "out"
// #define SYSFS_GPIO_RST_VAL "/sys/class/gpio/gpio440/value"
// #define SYSFS_GPIO_RST_VAL_H "1"
// #define SYSFS_GPIO_RST_VAL_L "0"

// int main()
// {
//     int fd;
//     int count = 30;

//     // 导出 GPIO 引脚
//     fd = open(SYSFS_GPIO_EXPORT, O_WRONLY);
//     if (fd == -1)
//     {
//         perror("ERR: export open error");
//         return EXIT_FAILURE;
//     }
//     write(fd, SYSFS_GPIO_RST_PIN_VAL, strlen(SYSFS_GPIO_RST_PIN_VAL));
//     close(fd);

//     // 设置 GPIO 方向为输出
//     fd = open(SYSFS_GPIO_RST_DIR, O_WRONLY);
//     if (fd == -1)
//     {
//         perror("ERR: direction open error");
//         return EXIT_FAILURE;
//     }
//     write(fd, SYSFS_GPIO_RST_DIR_VAL, strlen(SYSFS_GPIO_RST_DIR_VAL));
//     close(fd);

//     // 输出复位信号: 拉高 >100ns
//     fd = open(SYSFS_GPIO_RST_VAL, O_WRONLY);
//     if (fd == -1)
//     {
//         perror("ERR: gpio value open error");
//         return EXIT_FAILURE;
//     }
//     while (count)
//     {
//         count--;
//         printf("milk:%d\n", count);
        
//         // 拉高
//         write(fd, SYSFS_GPIO_RST_VAL_H, strlen(SYSFS_GPIO_RST_VAL_H));
//         usleep(100000); // 100ms 延时
        
//         // 拉低
//         write(fd, SYSFS_GPIO_RST_VAL_L, strlen(SYSFS_GPIO_RST_VAL_L));
//         usleep(100000); // 100ms 延时
//     }
//     close(fd);

//     // 取消导出 GPIO 引脚
//     fd = open(SYSFS_GPIO_UNEXPORT, O_WRONLY);
//     if (fd == -1)
//     {
//         perror("ERR: unexport open error");
//         return EXIT_FAILURE;
//     }
//     write(fd, SYSFS_GPIO_RST_PIN_VAL, strlen(SYSFS_GPIO_RST_PIN_VAL));
//     close(fd);

//     return 0;
// }

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#define GPIO_PIN "440"  // 使用字符串形式更方便
#define GPIO_PATH "/sys/class/gpio/gpio" GPIO_PIN
#define HIGH "1"
#define LOW "0"

void write_file(const char *path, const char *value) {
    int fd = open(path, O_WRONLY);
    if (fd == -1) {
        perror("Error opening file");
        exit(EXIT_FAILURE);
    }
    write(fd, value, strlen(value));
    close(fd);
}

int main() {
    // 导出GPIO
    write_file("/sys/class/gpio/export", GPIO_PIN);
    
    // 设置方向
    char dir_path[50];
    snprintf(dir_path, sizeof(dir_path), GPIO_PATH "/direction");
    write_file(dir_path, "out");

    // 控制GPIO
    char val_path[50];
    snprintf(val_path, sizeof(val_path), GPIO_PATH "/value");
    
    for (int i = 30; i > 0; i--) {
        printf("Countdown: %d\n", i);
        write_file(val_path, HIGH);
        usleep(100000);  // 100ms
        write_file(val_path, LOW);
        usleep(100000);  // 100ms
    }

    // 取消导出
    write_file("/sys/class/gpio/unexport", GPIO_PIN);

    return 0;
}