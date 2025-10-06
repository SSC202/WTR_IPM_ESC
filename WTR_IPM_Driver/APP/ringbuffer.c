#include "ringbuffer.h"
#include "string.h"

#define COMMAND_MIN_LENGTH 2   // 指令最小长度
#define BUFFER_SIZE        128 // 循环缓冲区大小

uint8_t buffer[BUFFER_SIZE]; // 循环缓冲区
uint8_t readIndex  = 0;      // 循环缓冲区读索引
uint8_t writeIndex = 0;      // 循环缓冲区写索引

/******************************************************************
 *                         循环缓冲区处理                         *
 *****************************************************************/

/**
 * @brief 增加读索引
 * @param length 要增加的长度
 */
static void command_add_readindex(uint8_t length)
{
    readIndex += length;
    readIndex %= BUFFER_SIZE;
}

/**
 * @brief 读取第i位数据,超过缓存区长度自动循环
 * @param i 要读取的数据索引
 */
static uint8_t command_read(uint8_t i)
{
    uint8_t index = (readIndex + i) % BUFFER_SIZE;
    return buffer[index];
}

/**
 * @brief 计算未处理的数据长度
 * @return 未处理的数据长度
 */
static uint8_t command_get_length()
{
    return (writeIndex + BUFFER_SIZE - readIndex) % BUFFER_SIZE;
}

/**
 * @brief 计算缓冲区剩余空间
 * @return 剩余空间
 */
static uint8_t command_get_remain()
{
    return BUFFER_SIZE - command_get_length();
}

/**
 * @brief 向缓冲区写入数据
 * @param data 要写入的数据指针
 * @param length 要写入的数据长度
 * @return 写入的数据长度
 */
uint8_t command_write(uint8_t *data, uint8_t length)
{
    // 如果缓冲区不足 则不写入数据 返回0
    if (command_get_remain() < length) {
        return 0;
    }
    // 使用memcpy函数将数据写入缓冲区
    if (writeIndex + length < BUFFER_SIZE) {
        memcpy(buffer + writeIndex, data, length);
        writeIndex += length;
    } else {
        uint8_t firstLength = BUFFER_SIZE - writeIndex;
        memcpy(buffer + writeIndex, data, firstLength);
        memcpy(buffer, data + firstLength, length - firstLength);
        writeIndex = length - firstLength;
    }
    return length;
}

/**
 * @brief 尝试获取一条指令
 * @param command 指令存放指针
 * @return 获取的指令长度
 * @retval 0 没有获取到指令
 */
uint8_t command_get_command(uint8_t *command)
{
    // 寻找完整指令
    while (1) {
        uint8_t dataLength = command_get_length();

        // 如果缓冲区长度小于COMMAND_MIN_LENGTH 则不可能有完整的指令
        if (dataLength < COMMAND_MIN_LENGTH) {
            return 0;
        }

        // 在缓冲区中查找\r\n结尾
        uint8_t found         = 0;
        uint8_t commandLength = 0;

        // 遍历缓冲区寻找\r\n
        for (uint8_t i = 0; i < dataLength - 1; i++) {
            if (command_read(i) == '\r' && command_read(i + 1) == '\n') {
                found         = 1;
                commandLength = i + 2; 
                break;
            }
        }

        // 如果没有找到完整的指令
        if (!found) {
            // 如果缓冲区已满，丢弃一个字节以防止死锁
            if (command_get_remain() == 0) {
                command_add_readindex(1);
                continue;
            }
            return 0;
        }

        // 如果找到完整指令,则将指令写入command
        for (uint8_t i = 0; i < commandLength; i++) {
            command[i] = command_read(i);
        }
        command_add_readindex(commandLength);
        return commandLength;
    }
}