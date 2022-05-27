//
//  Matrix4.swift
//  AC28Mac
//
//  Created by Edward Janne on 4/27/22.
//

/*
    Extensions to simd_double3x3 and simd_double4x4 to make operating on simd matrices more like traditional math.
*/

import Foundation
import simd

extension simd_double3x3 {
    // Constructor to create a rotation matrix given an axis of rotation and an angle
    init(rotationAxis axis: SIMD3<Double>, angle: Measurement<UnitAngle>) {
        let s = sin(angle.converted(to: .radians).value)
        let c = cos(angle.converted(to: .radians).value)
        let cc = 1.0 - c
        self.init(
            SIMD3<Double>(
                x: cc * axis.x * axis.x + c,
                y: cc * axis.x * axis.y + axis.z * s,
                z: cc * axis.x * axis.z - axis.y * s
            ),
            SIMD3<Double>(
                x: cc * axis.x * axis.y - axis.z * s,
                y: cc * axis.y * axis.y + c,
                z: cc * axis.y * axis.z + axis.x * s
            ),
            SIMD3<Double>(
                x: cc * axis.x * axis.z + axis.y * s,
                y: cc * axis.y * axis.z - axis.x * s,
                z: cc * axis.z * axis.z + c
            )
        )
    }
}

extension simd_double4x4 {
    // Constructor to create a rotation matrix given an axis of rotation and an angle
    init(rotationAxis axis: SIMD4<Double>, angle: Measurement<UnitAngle>) {
        let s = sin(angle.converted(to: .radians).value)
        let c = cos(angle.converted(to: .radians).value)
        let cc = 1.0 - c
        self.init(
            SIMD4<Double>(
                x: cc * axis.x * axis.x + c,
                y: cc * axis.x * axis.y + axis.z * s,
                z: cc * axis.x * axis.z - axis.y * s,
                w: 0.0
            ),
            SIMD4<Double>(
                x: cc * axis.x * axis.y - axis.z * s,
                y: cc * axis.y * axis.y + c,
                z: cc * axis.y * axis.z + axis.x * s,
                w: 0.0
            ),
            SIMD4<Double>(
                x: cc * axis.x * axis.z + axis.y * s,
                y: cc * axis.y * axis.z - axis.x * s,
                z: cc * axis.z * axis.z + c,
                w: 0.0
            ),
            SIMD4<Double>(
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0
            )
        )
    }
}

