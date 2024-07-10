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

struct ImmersiveView: View {
    
    @StateObject var model = HandTrackingViewModel()
    
    var body: some View {
        RealityView { content in
            // Add the initial RealityKit content
//            if let scene = try? await Entity(named: "Scene", in: realityKitContentBundle) {
//                content.add(scene)
//            }
            let skybox = createSkybox()
            content.add(skybox!)
            
//            content.add(model.setupContentEntity())
        }.task {
            
        }.task {
        
        }.task {
            
        }.gesture(SpatialTapGesture().targetedToAnyEntity().onEnded({value in
            Task {
                
            }
        }))
    }
    private func createSkybox() -> Entity? {
        let sphere = MeshResource.generateSphere(radius: 100)
        var skyboxMaterial = UnlitMaterial()
        
        //load image from the web
//        let imageURL = URL(string: "https://www.google.com/images/branding/googlelogo/1x/googlelogo_color_272x92dp.png")!
//        let imageData = try! Data(contentsOf: imageURL)
//        let image = UIImage(data: imageData)!
//        
        let texture = getTextureFromRemoteURL(URL(string: "https://media.discordapp.net/attachments/1237867213206061177/1242932373390102629/image.png?ex=666a007a&is=6668aefa&hm=21a5204719705fa479ce62206c16719885d30e756870fa3e7a1a3115e1313168&=&format=webp&quality=lossless")!)!
    
        skyboxMaterial.color = .init(texture: .init(texture))

        let skyboxEntity = Entity()
        skyboxEntity.components.set(ModelComponent(mesh: sphere, materials: [skyboxMaterial]))
        skyboxEntity.scale = .init(x: -1, y: 1, z: 1)
        
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

