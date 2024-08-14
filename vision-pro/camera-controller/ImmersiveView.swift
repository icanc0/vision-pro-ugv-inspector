//
//  ImmersiveView.swift
//  camera-controller
//
//  Created by Vincent Xie on 5/17/24.
//

import SwiftUI
import RealityKit
import RealityKitContent
import UIKit
import os

struct ImmersiveView: View {
    
//    @StateObject var model = HandTrackingViewModel()
    
    let logger = Logger(subsystem: "com.vincentxie.camera-controller", category: "ImmersiveView")
    
    @State private var skybox: Entity?
    
    var body: some View {
        VStack {
            RealityView { content in
                // Add the initial RealityKit content
    //            if let scene = try? await Entity(named: "Scene", in: realityKitContentBundle) {
    //                content.add(scene)
    //            }
                skybox = createSkybox()
                content.add(skybox!)
                logger.log("reality view setup done")
    //            content.add(model.setupContentEntity())
                
            }.gesture(
                SpatialTapGesture()
                    .targetedToAnyEntity()
                    .onEnded({value in
                        logger.log("inside gesture closure")
                        logger.log("tapped location \(value.location.debugDescription)")
                        logger.log("tapped location3d \(value.location3D.description)")
                    })
            )
        }
        
    }
    private func createSkybox() -> Entity? {
        let sphere = MeshResource.generateSphere(radius: 10)
        var skyboxMaterial = UnlitMaterial()
                
//        let texture = getTextureFromRemoteURL(URL(string: "http://192.168.0.135:8080")!)!
        
        let texture = getTextureFromRemoteURL(URL(string: "https://www.google.com/images/branding/googlelogo/1x/googlelogo_color_272x92dp.png")!)!
    
        skyboxMaterial.color = .init(texture: .init(texture))

        let skyboxEntity = Entity()
        skyboxEntity.components.set(ModelComponent(mesh: sphere, materials: [skyboxMaterial]))
        skyboxEntity.scale = .init(x: -1, y: 1, z: 1)
        
        
        logger.log("skybox created")

        return skyboxEntity
    }
    private func getTextureFromRemoteURL(_ remoteURL: URL) -> TextureResource? {
        let fileURL = FileManager.default.temporaryDirectory.appendingPathComponent(UUID().uuidString)
        
        let data = try! Data(contentsOf: remoteURL)
        
        try! data.write(to: fileURL)
        
        var texture: TextureResource? = nil
        do {
            texture = try TextureResource.load(contentsOf: fileURL)
            
        } catch {
            print("failed to load texture: \(error)")
            return nil
        }
        return texture
    }
    
}

#Preview(immersionStyle: .mixed) {
    ImmersiveView()
}

