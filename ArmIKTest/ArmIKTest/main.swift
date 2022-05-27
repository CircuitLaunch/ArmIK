//
//  main.swift
//  ArmIKTest
//
//  Created by Edward Janne on 5/26/22.
//

import Foundation

// Instantiate an IK computational object
let arm = ArmIK(upperLength: 100.0, lowerLength: 100.0, minimumDistanceFromShoulder: 25.0)

// Define variables to specify the angle and goal
var twist = Measurement<UnitAngle>(value: 45.0, unit: .degrees)
var goal = SIMD3<Double>(25.0, -100.0, 50.0)

for i in 0 ... 100 {
    // Animate the goal
    goal.y = -200.0 + Double(i)
    
    // Calculate the joint angles based on the twist and goal
    let angles = arm.perform(twist: twist, goal: goal)

    // Format the angles as strings for display
    let angle1 = String(format: "%.2fº", angles.theta0.converted(to: .degrees).value)
    let angle2 = String(format: "%.2fº", angles.theta1.converted(to: .degrees).value)
    let angle3 = String(format: "%.2fº", angles.theta2.converted(to: .degrees).value)
    let angle4 = String(format: "%.2fº", angles.theta3.converted(to: .degrees).value)
    
    // Print the results
    print("θ0: \(angle1), θ1: \(angle2), θ2: \(angle3), θ3: \(angle4)")
}
