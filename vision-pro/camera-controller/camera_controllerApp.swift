//
//  camera_controllerApp.swift
//  camera-controller
//
//  Created by Vincent Xie on 5/17/24.
//

import SwiftUI

@main
struct camera_controllerApp: App {

    var body: some Scene {
        WindowGroup {
            ContentView()
        }

        ImmersiveSpace(id: "CameraControllerSpace") {
            ImmersiveView()
        }.immersionStyle(selection: .constant(.full), in: .full)
    }
}
