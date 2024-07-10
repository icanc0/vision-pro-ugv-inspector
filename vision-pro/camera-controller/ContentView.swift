//
//  ContentView.swift
//  vlckit-test
//
//  Created by Vincent Xie on 5/31/24.
//

import SwiftUI
import RealityKit
import RealityKitContent
import AVKit
import WebKit

struct ContentView: View {
    
    @State private var showImmersiveSpace = false
    @State private var immersiveSpaceIsShown = false
    
    @Environment(\.openImmersiveSpace) var openImmersiveSpace
    @Environment(\.dismissImmersiveSpace) var dismissImmersiveSpace
    var timer = Timer()
    // Add a state variable for the HLS stream URL
//    @State private var streamURL = URL(string: "https://cph-p2p-msl.akamaized.net/hls/live/2000341/test/master.m3u8")!
//    @State private var streamURL = URL(string: "http://localhost:8888/proxied/index.m3u8")!
    @State private var player = AVPlayer(url: URL(string: "https://localhost:8888/proxied/index.m3u8")!)
    var body: some View {
        VStack {
//            Model3D(named: "Scene", bundle: realityKitContentBundle)
//                .padding(.bottom, 50)
            
//             Add the PlayerView here
//            VideoPlayerView(player: player)
//                .frame(height: 500)
            WebView(url: URL(string: "https://localhost:8888/proxied/")!)
            

            Toggle("Show ImmersiveSpace", isOn: $showImmersiveSpace)
                .font(.title)
                .frame(width: 360)
                .padding(24)
                .glassBackgroundEffect()
        }
        .padding()
        .task {
            Timer.scheduledTimer(withTimeInterval: 1, repeats: true, block: { _ in
                player.seekToLive()
            })
        }
        
        .onChange(of: showImmersiveSpace) { _, newValue in
            Task {
                if newValue {
                    switch await openImmersiveSpace(id: "CameraControllerSpace") {
                    case .opened:
                        immersiveSpaceIsShown = true
                    case .error, .userCancelled:
                        fallthrough
                    @unknown default:
                        immersiveSpaceIsShown = false
                        showImmersiveSpace = false
                    }
                } else if immersiveSpaceIsShown {
                    await dismissImmersiveSpace()
                    immersiveSpaceIsShown = false
                }
            }
        }
    }
    
}


#Preview(windowStyle: .automatic) {
    ContentView()
}


struct VideoPlayerView: UIViewControllerRepresentable {
    var player: AVPlayer
    func makeUIViewController(context: Context) -> AVPlayerViewController {
        let playerController = AVPlayerViewController()
        playerController.player = player
        
        player.play()
//        player.currentItem?.preferredForwardBufferDuration = 1

        player.isMuted = true
        return playerController
    }
    func updateUIViewController(_ uiViewController: AVPlayerViewController, context: Context) {
        player.seekToLive()
    }
}



extension AVPlayer {
    func seekToLive() {
        if let items = currentItem?.seekableTimeRanges, !items.isEmpty {
            let range = items[items.count - 1]
            let timeRange = range.timeRangeValue
            let startSeconds = CMTimeGetSeconds(timeRange.start)
            let durationSeconds = CMTimeGetSeconds(timeRange.duration)

            seek(to: CMTimeMakeWithSeconds(startSeconds + durationSeconds, preferredTimescale: 1))
        }
    }
}

struct WebView: UIViewRepresentable {
    let url: URL
    
    func makeUIView(context: Context) -> WKWebView {
        let webView = WKWebView()
        webView.navigationDelegate = context.coordinator
        return webView
    }
    
    func updateUIView(_ webView: WKWebView, context: Context) {
        let request = URLRequest(url: url)
        webView.load(request)
    }
    
    func makeCoordinator() -> WebViewCoordinator {
        WebViewCoordinator()
    }
}

class WebViewCoordinator: NSObject, WKNavigationDelegate {
    func webView(_ webView: WKWebView, decidePolicyFor navigationResponse: WKNavigationResponse, decisionHandler: @escaping (WKNavigationResponsePolicy) -> Void) {
        decisionHandler(.allow)
    }
    
    func urlSession(_ session: URLSession, didReceive challenge: URLAuthenticationChallenge, completionHandler: @escaping (URLSession.AuthChallengeDisposition, URLCredential?) -> Void) {
        // Trust the self-signed certificate
        let credential = URLCredential(trust: challenge.protectionSpace.serverTrust!)
        completionHandler(.useCredential, credential)
    }
}
