//
//  HandTrackingViewModel.swift
//  camera-controller
//
//  Created by Vincent Xie on 5/21/24.
//

import Foundation
import RealityKit
import RealityKitContent

import SwiftUI
import ARKit

@MainActor class HandTrackingViewModel: ObservableObject {
//    private let session = ARKitSession()
    private let handTracking = HandTrackingProvider()
    
    private var contentEntity = Entity()
  
    
    private let fingerEntities: [HandAnchor.Chirality: ModelEntity] = [
        .left: .createFingertip(),
        .right: .createFingertip()
    ]
    
    func setupContentEntity() -> Entity {
        for entity in fingerEntities.values {
            contentEntity.addChild(entity)
        }
        return contentEntity
    }
    
    func processHandUpdates() async {
        for await update in handTracking.anchorUpdates{
            let handAnchor = update.anchor
            
            guard handAnchor.isTracked else {continue}
            
            let fingerTip = handAnchor.handSkeleton?.joint(.indexFingerTip)
            
            guard ((fingerTip?.isTracked) != nil) else {continue}
            
            let originFromIndex = handAnchor.originFromAnchorTransform * fingerTip!.anchorFromJointTransform
            
        }
    }
}

extension ModelEntity {
    class func createFingertip() -> ModelEntity {
        let entity = ModelEntity(mesh: .generateSphere(radius: 0.005), materials: [UnlitMaterial(color: .blue)], collisionShape: .generateSphere(radius: 0.005), mass: 0.0)
       // entity.components.set(PhysicsBodyComponent(mode: .kinematic))
        return entity
    }
}
