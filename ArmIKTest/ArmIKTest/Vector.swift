//
//  Vector3.swift
//  AC28Mac
//
//  Created by Edward Janne on 4/27/22.
//

/*
    Extensions to simd_double3, SIMD3, simd_double4 and SIMD4 to make operating on simd vectors more like traditional math.
*/

import Foundation
import simd

infix operator •

extension simd_double3 {
    // The unit i, j, and k vectors
    static let i = SIMD3<Double>(1.0, 0.0, 0.0)
    static let j = SIMD3<Double>(0.0, 1.0, 0.0)
    static let k = SIMD3<Double>(0.0, 0.0, 1.0)
    
    // Return the magnitude
    var magnitude: Double {
        get {
            return simd_length(self)
        }
    }
    
    // Return a normalized copy
    var normalized: SIMD3<Double> {
        get {
            return simd_normalize(self)
        }
    }
    
    // Return the absolute angle between this and another vetor
    func angle(with v2: SIMD3<Double>)->Double {
        return acos((self • v2) / (magnitude * v2.magnitude))
    }
    
    // Return 1 if the angle between two vectors is less than, or equal to, 0
    // or -1 if the angle is greater than 0
    func sign(with v: SIMD3<Double>)->Double {
        let v1 = normalized
        let v2 = v.normalized
        let dot = v1 • v2
        if dot == 0.0 {
            return 1.0
        }
        return dot / abs(dot)
    }
    
    // Returns whether two vectors are coincident
    func isParallel(with v: SIMD3<Double>)->Bool {
        return ((self * v).magnitude == 0.0)
    }
}

extension SIMD3 {
    // Returns a string representation of the vector
    var toString: String {
        return "[\(x), \(y), \(z)]"
    }
}

// Prefix negation operator
prefix func -(v: SIMD3<Double>)->SIMD3<Double> {
    return SIMD3<Double>(-v.x, -v.y, -v.z)
}

// Infix equality operator
func == (v1: SIMD3<Double>, v2: SIMD3<Double>)->Bool {
    return
           abs(v1.x - v2.x) < 1E-5
        && abs(v1.y - v2.y) < 1E-5
        && abs(v1.z - v2.z) < 1E-5
}

// Infix inequality operator
func != (v1: SIMD3<Double>, v2: SIMD3<Double>)->Bool {
    return
           abs(v1.x - v2.x) > 1E-5
        || abs(v1.y - v2.y) > 1E-5
        || abs(v1.z - v2.z) > 1E-5
}

// Infix dot product operator
func •(v1: SIMD3<Double>, v2: SIMD3<Double>)->Double {
    return simd_dot(v1, v2)
}

// Infix cross product operator
func *(v1: SIMD3<Double>, v2: SIMD3<Double>)->SIMD3<Double> {
    return simd_cross(v1, v2)
}

// Infix scalar product assignment operator
func *=(v: inout SIMD3<Double>, s: Double) {
    v = simd_muladd(v, SIMD3<Double>(s, s, s), SIMD3<Double>(0.0, 0.0, 0.0))
}

// Infix scalar product operator
func *(v: SIMD3<Double>, s: Double)->SIMD3<Double> {
    return simd_muladd(v, SIMD3<Double>(s, s, s), SIMD3<Double>(0.0, 0.0, 0.0))
}

// Infix scalar product operator
func *(s: Double, v: simd_double3)->SIMD3<Double> {
    return simd_muladd(v, SIMD3<Double>(s, s, s), SIMD3<Double>(0.0, 0.0, 0.0))
}

// Infix scalar division assignment operator
func /=(v: inout simd_double3, s: Double) {
    let r = 1.0 / s
    v *= r
}

// Infix scalar division operator
func /(v: simd_double3, s: Double)->SIMD3<Double> {
    let r = 1.0 / s
    return v * r
}

// Infix addition assignment operator
func +=(v1: inout SIMD3<Double>, v2: SIMD3<Double>) {
    v1 = simd_muladd(v1, SIMD3<Double>(1.0, 1.0, 1.0), v2)
}

// Infix addition operator
func +(v1: SIMD3<Double>, v2: SIMD3<Double>)->SIMD3<Double> {
    return simd_muladd(v1, SIMD3<Double>(1.0, 1.0, 1.0), v2)
}

// Infix subtraction assignment operator
func -=(v1: inout SIMD3<Double>, v2: SIMD3<Double>) {
    v1 = simd_muladd(v2, SIMD3<Double>(-1.0, -1.0, -1.0), v1)
}

// Infix subtraction operator
func -(v1: SIMD3<Double>, v2: SIMD3<Double>)->SIMD3<Double> {
    return simd_muladd(v2, SIMD3<Double>(-1.0, -1.0, -1.0), v1)
}

// Infix components along/perpendicular to operator
func /(v1: SIMD3<Double>, v2: SIMD3<Double>)->(along: SIMD3<Double>, perpendicular: SIMD3<Double>) {
    let along = simd_project(v1, v2)
    return (along, v1 - along)
}

extension simd_double4 {
    // The unit i, j, and k vectors
    static let i = SIMD4<Double>(1.0, 0.0, 0.0, 0.0)
    static let j = SIMD4<Double>(0.0, 1.0, 0.0, 0.0)
    static let k = SIMD4<Double>(0.0, 0.0, 1.0, 0.0)
    
    // Return the magnitude
    var magnitude: Double {
        get {
            return simd_length(self)
        }
    }
    
    // Return a normalized copy
    var normalized: SIMD4<Double> {
        get {
            return simd_normalize(self)
        }
    }
    
    // Return the absolute angle between this and another vetor
    func angle(with v2: SIMD4<Double>)->Double {
        return acos((self • v2) / (magnitude * v2.magnitude))
    }
    
    // Return 1 if the angle between two vectors is less than, or equal to, 0
    // or -1 if the angle is greater than 0
    func sign(with v: SIMD4<Double>)->Double {
        let v1 = normalized
        let v2 = v.normalized
        if v1.x == -v2.x && v1.y == -v2.y && v1.z == -v2.z && v1.w == -v2.w {
            return -1.0
        }
        if v1.x == v2.x && v1.y == v2.y && v1.z == v2.z && v1.w == v2.w {
            return 1.0
        }
        return 0.0
    }
    
    // Returns whether two vectors are coincident
    func isParallel(with v: SIMD4<Double>)->Bool {
        return ((self * v).magnitude == 0.0)
    }
}

extension SIMD4 {
    // Returns a string representation of the vector
    var toString: String {
        return "[\(x), \(y), \(z), \(w)]"
    }
}

// Prefix negation operator
prefix func -(v: SIMD4<Double>)->SIMD4<Double> {
    return SIMD4<Double>(-v.x, -v.y, -v.z, -v.w)
}

// Infix equality operator
func == (v1: SIMD4<Double>, v2: SIMD4<Double>)->Bool {
    return simd_equal(v1, v2)
}

// Infix inequality operator
func != (v1: SIMD4<Double>, v2: SIMD4<Double>)->Bool {
    return !simd_equal(v1, v2)
}

// Infix dot product operator
func •(v1: SIMD4<Double>, v2: SIMD4<Double>)->Double {
    return simd_dot(v1, v2)
}

// Infix cross product operator
func *(v1: SIMD4<Double>, v2: SIMD4<Double>)->SIMD4<Double> {
    return simd_make_double4(simd_cross(simd_make_double3(v1), simd_make_double3(v2)))
}

// Infix scalar product assignment operator
func *=(v: inout SIMD4<Double>, s: Double) {
    v = simd_muladd(v, SIMD4<Double>(s, s, s, 0.0), SIMD4<Double>(0.0, 0.0, 0.0, 0.0))
}

// Infix scalar product operator
func *(v: SIMD4<Double>, s: Double)->SIMD4<Double> {
    return simd_muladd(v, SIMD4<Double>(s, s, s, 0.0), SIMD4<Double>(0.0, 0.0, 0.0, 0.0))
}

// Infix scalar product operator
func *(s: Double, v: SIMD4<Double>)->SIMD4<Double> {
    return simd_muladd(v, SIMD4<Double>(s, s, s, 0.0), SIMD4<Double>(0.0, 0.0, 0.0, 0.0))
}

// Infix scalar division assignment operator
func /=(v: inout SIMD4<Double>, s: Double) {
    let r = 1.0 / s
    v *= r
}

// Infix scalar division operator
func /(v: SIMD4<Double>, s: Double)->SIMD4<Double> {
    let r = 1.0 / s
    return v * r
}

// Infix addition assignment operator
func +=(v1: inout SIMD4<Double>, v2: SIMD4<Double>) {
    v1 = simd_muladd(v1, SIMD4<Double>(1.0, 1.0, 1.0, 0.0), v2)
}

// Infix addition operator
func +(v1: SIMD4<Double>, v2: SIMD4<Double>)->SIMD4<Double> {
    return simd_muladd(v1, SIMD4<Double>(1.0, 1.0, 1.0, 0.0), v2)
}

// Infix subtraction assignment operator
func -=(v1: inout SIMD4<Double>, v2: SIMD4<Double>) {
    v1 = simd_muladd(v2, SIMD4<Double>(-1.0, -1.0, -1.0, 0.0), v1)
}

// Infix subtraction operator
func -(v1: SIMD4<Double>, v2: SIMD4<Double>)->SIMD4<Double> {
    return simd_muladd(v2, SIMD4<Double>(-1.0, -1.0, -1.0, 0.0), v1)
}

// Infix components along/perpendicular to operator
func /(v1: SIMD4<Double>, v2: SIMD4<Double>)->(along: SIMD4<Double>, perpendicular: SIMD4<Double>) {
    let along = simd_project(v1, v2)
    return (along, v1 - along)
}
