/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 * @brief Defines the Byte class.
 * 定义Byte类。
 */

#pragma once

#include <string>

/**
 * @namespace apollo::drivers::canbus
 * @brief apollo::drivers::canbus
 */
namespace apollo {
namespace drivers {
namespace canbus {

/**
 * @class Byte  Byte类
 * @brief The class of one byte, which is 8 bits.
 *        It includes some operations on one byte.
 * 一个字节的类，为8位。 它包括对一个字节的一些操作。
 */
class Byte {
 public:
  /**
   * @brief Constructor which takes a pointer to a one-byte unsigned integer.
   * 构造函数：采用一个指向一个字节无符号整数的指针。
   * @param value The pointer to a one-byte unsigned integer for construction.
   * 指向要构造的一个字节无符号整数的指针。
   */
  //C++中的explicit关键字只能用于修饰只有一个参数的类构造函数, 它的作用是表明该构造函数是显示的, 而非隐式的.
  // 跟它相对应的另一个关键字是implicit, 意思是隐藏的,类构造函数默认情况下即声明为implicit(隐式).
  explicit Byte(const uint8_t *value);

  /**
   * @brief Constructor which takes a reference to a one-byte unsigned integer.
   * 构造函数，引用一字节无符号整数。
   * @param value The reference to a one-byte unsigned integer for construction.
   * 对用于构造的一字节无符号整数的引用。
   */
  Byte(const Byte &value);

  /**
   * @brief Desctructor.
   */
  ~Byte() = default;  //析构函数

  /**
   * @brief Transform an integer with the size of one byte to its hexadecimal
   *        represented by a string.
   * 将一个字节大小的整数转换为由字符串表示的十六进制。  hexadecimal十六进制
   * @param value The target integer to transform.
   * 要转换的目标整数。
   * @return Hexadecimal representing the target integer.
   * 返回代表目标整数的十六进制。  integer整数
   */
  static std::string byte_to_hex(const uint8_t value);

  /**
   * @brief Transform an integer with the size of 4 bytes to its hexadecimal
   *        represented by a string.
   * 将大小为4个字节的整数转换为由字符串表示的十六进制。
   * @param value The target integer to transform.
   * 要转换的目标整数。
   * @return Hexadecimal representing the target integer.
   * 返回代表目标整数的十六进制。
   */
  static std::string byte_to_hex(const uint32_t value);

  /**
   * @brief Transform an integer with the size of one byte to its binary
   *        represented by a string.
   * 将一个字节大小的整数转换为由字符串表示的二进制数。  binary二进制
   * @param value The target integer to transform.
   * 要转换的目标整数。
   * @return Binary representing the target integer.
   * 返回代表目标整数的二进制。
   */
  static std::string byte_to_binary(const uint8_t value);

  /**
   * @brief Set the bit on a specified position to one.
   * 将指定位置的位设置为1。
   * @param pos The position of the bit to be set to one.
   * 要设置为1的位的位置。
   */
  void set_bit_1(const int32_t pos);

  /**
   * @brief Set the bit on a specified position to zero.
   * 将指定位置的位设置为零。
   * @param pos The position of the bit to be set to zero.
   * 要设置为0的位的位置。
   */
  void set_bit_0(const int32_t pos);

  /**
   * @brief Check if the bit on a specified position is one.
   * 检查指定位置的位是否为1。
   * @param pos The position of the bit to check.
   * 要检查的位的位置。
   * @return If the bit on a specified position is one.
   * 返回指定位置的位是否为1。
   */
  bool is_bit_1(const int32_t pos) const;

  /**
   * @brief Reset this Byte by a specified one-byte unsigned integer.
   * 用指定的一字节无符号整数重置此字节。
   * @param value The one-byte unsigned integer to set this Byte.
   * 设置此字节的一字节无符号整数。
   */
  void set_value(const uint8_t value);

  /**
   * @brief Reset the higher 4 bits as the higher 4 bits of a specified one-byte
   *        unsigned integer.
   * 将高4位重置为指定的1字节无符号整数的高4位。
   * @param value The one-byte unsigned integer whose higher 4 bits are used to
   *        set this Byte's higher 4 bits.
   * 一字节无符号整数，其高4位用于设置该字节的高4位。
   */
  void set_value_high_4_bits(const uint8_t value);

  /**
   * @brief Reset the lower 4 bits as the lower 4 bits of a specified one-byte
   *        unsigned integer.
   * 将低4位重置为指定的1字节无符号整数的低4位。
   * @param value The one-byte unsigned integer whose lower 4 bits are used to
   *        set this Byte's lower 4 bits.
   * 一字节无符号整数，其低4位用于设置该字节的低4位。
   */
  void set_value_low_4_bits(const uint8_t value);

  /**
   * @brief Reset some consecutive bits starting from a specified position with
   *        a certain length of another one-byte unsigned integer.
   * 从指定位置开始以另一个一字节无符号整数的一定长度重置一些连续的位。
   * @param value The one-byte unsigned integer whose certain bits are used
   *        to set this Byte.
   * 一字节无符号整数，其某些位用于设置此字节。
   * @param start_pos The starting position (from the lowest) of the bits.
   * 位的起始位置（从最低位置开始）。
   * @param length The length of the consecutive bits.
   * 连续位的长度。
   */
  void set_value(const uint8_t value, const int32_t start_pos,
                 const int32_t length);

  /**
   * @brief Get the one-byte unsigned integer.
   * 获取一字节无符号整数。
   * @return The one-byte unsigned integer.
   * 返回一字节无符号整数。
   */
  uint8_t get_byte() const;

  /**
   * @brief Get a one-byte unsigned integer representing the higher 4 bits.
   * 获取一个代表高4位的一字节无符号整数。
   * @return The one-byte unsigned integer representing the higher 4 bits.
   * 返回一字节无符号整数，代表较高的4位。
   */
  uint8_t get_byte_high_4_bits() const;

  /**
   * @brief Get a one-byte unsigned integer representing the lower 4 bits.
   * 获取一个代表低四位的一字节无符号整数。
   * @return The one-byte unsigned integer representing the lower 4 bits.
   * 返回代表低四位的一字节无符号整数。
   */
  uint8_t get_byte_low_4_bits() const;

  /**
   * @brief Get a one-byte unsigned integer representing the consecutive bits
   *        from a specified position (from lowest) by a certain length.
   * 获取一个一字节的无符号整数，该整数表示从指定位置（从最低位置开始）一定长度的连续位。
   * @param start_pos The starting position (from lowest) of bits.
   * 位的起始位置（从最低开始）。
   * @param length The length of the selected consecutive bits.
   * 所选连续位的长度。
   * @return The one-byte unsigned integer representing the selected bits.
   * 返回代表所选位的一字节无符号整数。
   */
  uint8_t get_byte(const int32_t start_pos, const int32_t length) const;

  /**
   * @brief Transform to its hexadecimal represented by a string.
   * 转换为由字符串表示的十六进制。
   * @return Hexadecimal representing the Byte.
   * 返回代表字节的十六进制。
   */
  std::string to_hex_string() const;

  /**
   * @brief Transform to its binary represented by a string.
   * 转换为由字符串表示的二进制。
   * @return Binary representing the Byte.
   * 返回二进制表示字节。
   */
  std::string to_binary_string() const;

 private:
  uint8_t *value_; //一字节无符号整数指针
};

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
