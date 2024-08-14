//
//  camera_controllerApp.swift
//  camera-controller
//
//  Created by Vincent Xie on 5/17/24.
//

import SwiftUI
import os
@main
struct camera_controllerApp: App {
    
    
    let logger = Logger(subsystem: "com.vincentxie.camera-controller", category : "camera_controllerApp")
    
    var body: some Scene {
        WindowGroup {
            ContentView()
        }

        ImmersiveSpace(id: "CameraControllerSpace") {
            ImmersiveView()
                .gesture(
                    SpatialTapGesture()
                        .onEnded { value in
                            logger.log("omg its from the main immersive view")

                        }
                        
                )
        }.immersionStyle(selection: .constant(.full), in: .full)
    }
}
