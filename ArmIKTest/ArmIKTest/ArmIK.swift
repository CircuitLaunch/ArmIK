//
//  ArmIK.swift
//  AC28Mac
//
//  Created by Edward Janne on 4/27/22.
//

import Foundation
import simd

class ArmIK {

    //============================================================================
    // NOTE
    //
    //  I have extended the standard classes CGPoint, simd_double3, simd_double4
    //  simd_double3x3 and simd_double4x4 with custom operators and getter
    //  properties for improved legibility. Please refer to the files
    //  CGPoint.swift, Matrix.swift, and Vector.swift.
    //============================================================================
    
    // Length of the upper arm
    var l1: Double
    // Length of the lower arm
    var l2: Double
    // Minimum distance between the end effector and the shoulder joint
    var minDist: Double
    
    init(upperLength l1: Double, lowerLength l2: Double, minimumDistanceFromShoulder min: Double) {
        self.l1 = l1
        self.l2 = l2
        self.minDist = min
    }
    
    static func map(_ tuple: (theta0: Measurement<UnitAngle>, theta1: Measurement<UnitAngle>, theta2: Measurement<UnitAngle>, theta3: Measurement<UnitAngle>), _ closure: (Measurement<UnitAngle>)->(Measurement<UnitAngle>))->(theta0: Measurement<UnitAngle>, theta1: Measurement<UnitAngle>, theta2: Measurement<UnitAngle>, theta3: Measurement<UnitAngle>) {
        return (
            theta0: closure(tuple.theta0),
            theta1: closure(tuple.theta1),
            theta2: closure(tuple.theta2),
            theta3: closure(tuple.theta3))
    }
    
    func perform(twist: Measurement<UnitAngle>, goal: SIMD3<Double>, trace: Bool = false)->(theta0: Measurement<UnitAngle>, theta1: Measurement<UnitAngle>, theta2: Measurement<UnitAngle>, theta3: Measurement<UnitAngle>) {
        /*
        
         j0
         *--* j1
            |\
            | \
            |  * j2 (twist)
            |   \
            |    \
          u |     \
            |      \
            |       \
            |        \
          m +---------* j3
            |   v    /
            |       /
            |      /
            |     /
            |    /
            |   /
            |  /
            | /
            |/
            * j4
            
        */
        
        // **NOTE**
        // Assuming that j1 is the origin
        
        var j1j4 = goal
        var projj1j4 = j1j4
        projj1j4.x = 0.0
        
        // Minimum distance between j1 and j4
        if goal.magnitude < minDist {
            j1j4 = goal.normalized * minDist
        }
        
        // Maximum distance is the length of the upperarm + the length of the lowerarm
        if goal.magnitude > (l1 + l2) {
            j1j4 = goal.normalized * (l1 + l2)
        }
        
        // Variables to hold the computed angles
        var theta0 = Measurement(value: 0.0, unit: UnitAngle.radians)
        var theta1 = Measurement(value: 0.0, unit: UnitAngle.radians)
        var theta2 = Measurement(value: 0.0, unit: UnitAngle.radians)
        var theta3 = Measurement(value: 0.0, unit: UnitAngle.radians)
        
        // Magnitude of j1j4
        let magj1j4 = Double(j1j4.magnitude)
        
        // This only makes sense if goal is within reach
        if magj1j4 < l1 + l2 {
            
            //---------------------------------------------------------
            // Calculating theta3 (the elbow) using the law of cosines
            //---------------------------------------------------------
            let numerator = magj1j4 * magj1j4 - l1 * l1 - l2 * l2
            let denominator = -2.0 * l1 * l2
            theta3.value = Double.pi - acos( numerator / denominator )

            //-------------------------------------------
            // Calculating the position of j3 (the elbow)
            //-------------------------------------------
            
            // Normalized vector from origin to end effector
            let unitj1j4 = j1j4.normalized
            
            /*
            
                Heron's formula for the area of a triangle given its side lengths
                
                               A = sqrt( s (s - |J3 - J1|) (s - |J4 - J3|) (s - |J4 - J1|) )
                            
                where s is half the perimeter
                
                    |J4 - J1||v| = 2A
                
                                    2.0 sqrt( s (s - |J3 - J1|) (s - |J4 - J3|) (s - |J4 - J1|) )
                             |v| =  -------------------------------------------------------------
                                                             |J4 - J1|
                                                    
                    
                       |J3 - J1| = sqrt( |u|^2 + |v|^2 )
                             |u| = sqrt( |J3 - J1|^2 - |v|^2 )
                
            */
            
            // 0.5 * Perimeter of triangle (J1 J3 J4)
            let s = (l1 + l2 + magj1j4) * 0.5
            
            // Magnitude of vector v (mj3)
            let magvsq = 4.0 * (s * (s - l1) * (s - l2) * (s - magj1j4)) / (j1j4 • j1j4)
                                                        
            // Use Pythagoras' theorem to derive vector u (j1m)
            let magu = sqrt(l1 * l1 - magvsq)
            let u = unitj1j4 * magu
            
            var unitv: SIMD3<Double> = .zero
            // Use cross-products to find vector v (mj3)
            // If j1j4 coincides with unit i vector, use the unit j vector instead
            if j1j4.isParallel(with: simd_double3.i) {
                // Cross product of j1j4 and unit j gives vector perpendicular
                //     to plane of triangle j1j3j4 before applying twist
                // Crossing that again with j1j4 gives vector along v
                unitv = ((unitj1j4 * simd_double3.j) * unitj1j4).normalized
            }  else {
                // Cross product of j1j4 and unit i gives vector perpendicular
                //     to plane of triangle j1j3j4 before applying twist
                // Crossing that again with j1j4 gives vector along v
                unitv = ((unitj1j4 * simd_double3.i) * unitj1j4).normalized
            }
            
            // Use magnitude to derive v0 (v before applying twist)
            let v0 = unitv * sqrt(magvsq)
            
            // j3 before the twist
            let j1j30 = u + v0
            
            // The dot product can act as an indicator as to whether
            // the angle between two vectors is acute (+ve) or obtuse (-ve).
            let dot = (projj1j4.normalized) • (j1j30.normalized)
            
            // Here, I'm using it to determine whether the elbow is
            // "above" the shoulder which means that the polarity
            // of the rotation would flip.
            let shoulderRollSign = (dot != 0.0) ? dot / abs(dot) : 1.0
            
            // Generate an axis/angle rotation matrix to rotate around the j1j4 vector by twist raidans
            let twistM = simd_double3x3(rotationAxis: unitj1j4, angle: twist)
            
            // Rotate to account for arm twist
            let v = twistM * v0
            
            // Vector sum of u and v gives the final position of elbow
            let j1j3 = u + v
            
            //------------------------------------
            // Calculating theta2 (the arm twist)
            //------------------------------------
            
            // perp is the normal to the plane defined by j1j4 and j1j3
            let perp = (j1j4 * j1j3).normalized
            
            // Axis of rotation of j1 after twist
            let axisj1 = (j1j3 * simd_double3.i).normalized * shoulderRollSign
            
            // perp and axisj1 are mutually orthogonal to j1j3, so their
            // cross-product must be parallel with j1j3
            
            if axisj1 != perp { // Unless they are coincident, in which case no need to adjust for sign
                // Factor indicating the orientation of the twist
                let twistSign = (axisj1 * perp).sign(with: j1j3)
                // The angle between two vectors is unsigned, so must use orientation to discover sign
                theta2.value = axisj1.angle(with: perp) * twistSign
            }
            
            //----------------------------------------
            // Calculating theta1 (the shoulder roll)
            //----------------------------------------
            
            // Signed angle of the shoulder roll
            theta1.value = j1j3.angle(with: simd_double3.i) * shoulderRollSign
            
            //---------------------------------------
            // Calculating theta0 (the should pitch)
            //---------------------------------------
            
            // Project the vector to the elbow on to the plane perpendicular to shoulder pitch, i.e. the yz-plane
            var projj1j3 = j1j3
            projj1j3.x = 0.0
            
            // The cross product of -j with the above vector produces a vector parallel to the i vector.
            // If it is coincident with i, the rotation direction is positive, if opposite, the rotation
            // direction is negative.
            let pitchSign = ((projj1j3.z != 0.0) ? ((-simd_double3.j) * projj1j3).sign(with: simd_double3.i) : 1.0)
            
            // Signed angle of the shoulder pitch
            theta0.value = projj1j3.angle(with: -simd_double3.j) * pitchSign
            
            // If the elbow is "above" the shoulder, the above calculation produces a flipped rotation.
            // Flip the angle by pi radians to compensate
            if shoulderRollSign < 0 {
                theta0.value += .pi
            }
            
        // If the arm is fully extended, theta3 is zero
        } else {
            // theta2 is just the given arm twist
            theta2.value = twist.converted(to: .radians).value
            
            // theta1 is the angle of j1j4 with i
            theta1.value = j1j4.angle(with: simd_double3.i)
            
            // The cross product of -j with the projection of j1j4 on to the yz-plane produces a vector parallel
            // to the i vector. If it is coincident with i, the rotation direction is positive, if opposite, the
            // rotation direction is negative.
            let pitchSign = ((projj1j4.z != 0.0) ? ((-simd_double3.j) * projj1j4).sign(with: simd_double3.i) : 1.0)
            
            // Signed angle of the shoulder pitch
            theta0.value = projj1j4.angle(with: -simd_double3.j) * pitchSign
        }
                
        return (
            theta0: theta0,
            theta1: theta1,
            theta2: theta2,
            theta3: theta3)
    }
}
