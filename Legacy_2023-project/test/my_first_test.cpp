/*
 * Copyright (c) 2020-2021 Legacy_Robotics
 *
 * This file is part of Legacy_2023.
 *
 * Legacy_2023 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Legacy_2023 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Legacy_2023.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>

#include "my_first_file.hpp"

TEST(hello, world) { EXPECT_EQ(3, sum(1, 2)); }
