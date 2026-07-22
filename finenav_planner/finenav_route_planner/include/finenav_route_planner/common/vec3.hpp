// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once
#include <cstddef>
#include <cmath>

namespace finenav {

    /**
     * @brief 整数三维栅格坐标
     * @details 搜索算法内部使用的基础坐标类型；
     *          二维栅格场景下 z 恒为 0，可无缝复用三维实现
     */
    struct Vec3 {
        int x, y, z;   // 栅格索引分量

        /// 构造函数（默认为原点 (0, 0, 0)）
        Vec3(int x = 0, int y = 0, int z = 0) : x(x), y(y), z(z) {}

        /// 逐分量相等比较
        bool operator==(const Vec3& other) const {
            return x == other.x && y == other.y && z == other.z;
        }

        /// 逐分量相加（用于邻域偏移）
        Vec3 operator+(const Vec3& other) const {
            return Vec3{x + other.x, y + other.y, z + other.z};
        }

        /// 到 other 的欧氏距离
        double distance(const Vec3& other) const {
            double dx = x - other.x;
            double dy = y - other.y;
            double dz = z - other.z;
            return std::sqrt(dx*dx + dy*dy + dz*dz);
        }

        /// 到 other 的曼哈顿距离
        int manhattan(const Vec3& other) const {
            return std::abs(x - other.x) + std::abs(y - other.y) + std::abs(z - other.z);
        }
    };

    /**
     * @brief Vec3 的哈希函数对象（用于 unordered_map / unordered_set）
     * @note 采用类 FNV 的位混合方式组合三个分量
     */
    struct Vec3Hash {
        std::size_t operator()(const Vec3& v) const noexcept {
            // 类 FNV 的位混合组合
            std::size_t h = static_cast<std::size_t>(v.x);
            h ^= (static_cast<std::size_t>(v.y) << 20) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= (static_cast<std::size_t>(v.z) << 40) + 0x9e3779b9 + (h << 6) + (h >> 2);
            return h;
        }
    };

} // namespace finenav

