//
//  CGPoint.swift
//  AC28
//
//  Created by Edward Janne on 5/8/22.
//

/*
    Extensions to CGPoint to make operating on them more like traditional math.
*/

import Foundation

#if os(iOS)
    import UIKit
#endif

extension CGPoint {
    // Magnitude
    var magnitude: CGFloat {
        get {
            return sqrt(x * x + y*y)
        }
        set {
            let mag = 1.0 / magnitude
            x *= mag * newValue
            y *= mag * newValue
        }
    }
    
    // Normalized copy
    var normalized: CGPoint {
        return self / magnitude
    }
    
    // Angle made with another vector
    func angle(with: CGPoint)->CGFloat {
        return acos((self • with) / (magnitude * with.magnitude))
    }
    
    // Return a string representation
    var toString: String {
        return "[\(x), \(y)]"
    }
}

// Infix scalar product assignment operator
@discardableResult func *=(_ p: inout CGPoint, _ s: CGFloat)->CGPoint {
    p.x *= s
    p.y *= s
    return p
}

// Infix scalar division assignment operator
@discardableResult func /=(_ p: inout CGPoint, _ s: CGFloat)->CGPoint {
    p.x /= s
    p.y /= s
    return p
}

// Infix addition assignment operator
@discardableResult func +=(_ p1: inout CGPoint, _ p2: CGPoint)->CGPoint {
    p1.x += p2.x
    p1.y += p2.y
    return p1
}

// Infix subtraction assignment operator
@discardableResult func -=(_ p1: inout CGPoint, _ p2: CGPoint)->CGPoint {
    p1.x -= p2.x
    p1.y -= p2.y
    return p1
}

infix operator •

// Infix dot product operator
func •(_ p1: CGPoint, _ p2: CGPoint)->CGFloat {
    p1.x * p2.x + p1.y * p2.y
}

// Infix cross product operator (just returns the Z value)
func *(_ p1: CGPoint, _ p2: CGPoint)->CGFloat {
    return p1.x * p2.y - p1.y * p2.x
}

// Infix scalar product operator
func *(_ p: CGPoint, _ s: CGFloat)->CGPoint {
    return CGPoint(x: p.x * s, y: p.y * s)
}

// Infix scalar division operator
func /(_ p: CGPoint, _ s: CGFloat)->CGPoint {
    return CGPoint(x: p.x / s, y: p.y / s)
}

// Infix addition operator
func +(_ p1: CGPoint, _ p2: CGPoint)->CGPoint {
    return CGPoint(x: p1.x + p2.x, y: p1.y + p2.y)
}

// Infix subtraction operator
func -(_ p1: CGPoint, _ p2: CGPoint)->CGPoint {
    return CGPoint(x: p1.x - p2.x, y: p1.y - p2.y)
}
