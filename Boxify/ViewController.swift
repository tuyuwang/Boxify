//
//  ViewController.swift
//  AR Testbed
//
//  Created by Alun Bestor on 2017-06-14.
//  Copyright © 2017 Alun Bestor. All rights reserved.
//

import UIKit
import SceneKit
import ARKit

class ViewController: UIViewController, ARSCNViewDelegate, UIGestureRecognizerDelegate {
    
    @IBOutlet var sceneView: ARSCNView!
    
    var panGesture: UIPanGestureRecognizer!
    var doubleTapGesture: UITapGestureRecognizer!
    var rotationGesture: UIRotationGestureRecognizer!
    var tapGesture: UITapGestureRecognizer!
    
    var box: Box!
    var hitTestPlane: SCNNode!
    var floor: SCNNode!
    
    var currentAnchor: ARAnchor?
    
    private(set) var pointCloud: ScannedPointCloud!
    var timeOfLastReferenceObjectCreation = CACurrentMediaTime()

    var tapA: SCNNode!
    var tapMode: Bool = false {
        didSet {
            if tapMode {
                tapA.isHidden = false
            } else {
                tapA.isHidden = true
            }
    
        }
    }
    
    lazy var resultLabel: UILabel = {
        let label = UILabel()
        label.frame = CGRect(x: 12, y: 45, width: view.frame.width - 12 * 2, height: 130)
        label.numberOfLines = 0
        label.font = UIFont.boldSystemFont(ofSize: 24)
        label.textColor = .red
        return label
    }()
    
    lazy var lengthFormatter: NumberFormatter = {
        let formatter = NumberFormatter()
        formatter.numberStyle = .decimal
        formatter.maximumFractionDigits = 1
        formatter.multiplier = 100
        return formatter
    }()
    
    struct RenderingCategory: OptionSet {
        let rawValue: Int
        static let reflected = RenderingCategory(rawValue: 1 << 1)
        static let planes = RenderingCategory(rawValue: 1 << 2)
    }
    
    enum InteractionMode {
        case waitingForLocation
        case draggingInitialWidth, draggingInitialLength
        case waitingForFaceDrag, draggingFace(side: Box.Side, dragStart: SCNVector3)
    }
    
    enum PointCloudMode {
        case none
        case point
        case line
        case box
    }
    
    var cloudMode: PointCloudMode = .none
    
    var planesShown: Bool {
        get { return RenderingCategory(rawValue: sceneView.pointOfView!.camera!.categoryBitMask).contains(.planes) }
        set {
            var mask = RenderingCategory(rawValue: sceneView.pointOfView!.camera!.categoryBitMask)
            if newValue == true {
                mask.formUnion(.planes)
            } else {
                mask.subtract(.planes)
            }
            sceneView.pointOfView!.camera!.categoryBitMask = mask.rawValue
        }
    }
    
    var mode: InteractionMode = .waitingForLocation {
        didSet {
            switch mode {
            case .waitingForLocation:
                rotationGesture.isEnabled = false
                
                box.isHidden = true
                box.clearHighlights()
                
                hitTestPlane.isHidden = true
                floor.isHidden = true
                
                planesShown = true
                
            case .draggingInitialWidth, .draggingInitialLength:
                rotationGesture.isEnabled = true
                
                box.isHidden = false
                box.clearHighlights()
                
                floor.isHidden = false
                
                // Place the hit-test plane flat on the z-axis, aligned with the bottom of the box.
                hitTestPlane.isHidden = false
                hitTestPlane.position = .zero
                hitTestPlane.boundingBox.min = SCNVector3(x: -1000, y: 0, z: -1000)
                hitTestPlane.boundingBox.max = SCNVector3(x: 1000, y: 0, z: 1000)
                
                planesShown = false
                
            case .waitingForFaceDrag:
                rotationGesture.isEnabled = true
                
                box.isHidden = false
                box.clearHighlights()
                
                floor.isHidden = false
                hitTestPlane.isHidden = true
                
                planesShown = false
                
            case .draggingFace(let side, let dragStart):
                rotationGesture.isEnabled = true
                
                box.isHidden = false
                floor.isHidden = false
                
                hitTestPlane.isHidden = false
                hitTestPlane.position = dragStart
                
                planesShown = false
                
                box.highlight(side: side)
                
                // Place the hit-test plane straight through the dragged side, centered at the point on which the drag started.
                // This makes the drag operation act as though you're dragging that exact point on the side to a new location.
                // TODO: the plane should be constrained so that it always rotates to face the camera along the axis that goes through the dragged side.
                switch side.axis {
                case .x:
                    hitTestPlane.boundingBox.min = SCNVector3(x: -1000, y: -1000, z: 0)
                    hitTestPlane.boundingBox.max = SCNVector3(x: 1000, y: 1000, z: 0)
                case .y:
                    hitTestPlane.boundingBox.min = SCNVector3(x: -1000, y: -1000, z: 0)
                    hitTestPlane.boundingBox.max = SCNVector3(x: 1000, y: 1000, z: 0)
                case .z:
                    hitTestPlane.boundingBox.min = SCNVector3(x: 0, y: -1000, z: -1000)
                    hitTestPlane.boundingBox.max = SCNVector3(x: 0, y: 1000, z: 1000)
                }
            }
        }
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // Set the view's delegate
        sceneView.delegate = self
        sceneView.session.delegate = self
//        sceneView.debugOptions = .showFeaturePoints
        
        sceneView.antialiasingMode = .multisampling4X
        sceneView.autoenablesDefaultLighting = true
        
        panGesture = UIPanGestureRecognizer(target: self, action: #selector(handlePan))
        
        doubleTapGesture = UITapGestureRecognizer(target: self, action: #selector(handleDoubleTap))
        doubleTapGesture.numberOfTapsRequired = 2
        
        rotationGesture = UIRotationGestureRecognizer(target: self, action: #selector(handleRotation))
        
        tapGesture = UITapGestureRecognizer(target: self, action: #selector(handleTap))
        tapGesture.require(toFail: doubleTapGesture)
        
        sceneView.addGestureRecognizer(panGesture)
        sceneView.addGestureRecognizer(doubleTapGesture)
        sceneView.addGestureRecognizer(rotationGesture)
        sceneView.addGestureRecognizer(tapGesture)
        sceneView.addSubview(resultLabel)
        
        box = Box()
        box.isHidden = true
        box.delegate = self
        sceneView.scene.rootNode.addChildNode(box)
        
        tapA = self.makeVertex()
                
        // Create an invisible plane used for hit-testing during drag operations.
        // This is a child of the box, so it inherits the box's own transform.
        // It is resized and repositioned within the box depending on what part of the box is being dragged.
        hitTestPlane = SCNNode()
        hitTestPlane.isHidden = true
        box.addChildNode(hitTestPlane)
        
        let floorSurface = SCNFloor()
        floorSurface.reflectivity = 0.2
        floorSurface.reflectionFalloffEnd = 0.05
        floorSurface.reflectionCategoryBitMask = RenderingCategory.reflected.rawValue
        
        // Floor scene reflections are blended with the diffuse color's transparency mask, so if diffuse is transparent then no reflection will be shown.
        // To get around this, we make the floor black and use additive blending so that only the brighter reflection is shown.
        floorSurface.firstMaterial?.diffuse.contents = UIColor.black
        floorSurface.firstMaterial?.writesToDepthBuffer = false
        floorSurface.firstMaterial?.blendMode = .add
        
        floor = SCNNode(geometry: floorSurface)
        floor.isHidden = true
        
        box.addChildNode(floor)
        box.categoryBitMask |= RenderingCategory.reflected.rawValue
        
        pointCloud = ScannedPointCloud()
        sceneView.scene.rootNode.addChildNode(pointCloud)
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        // Create a session configuration
        
        let configuration = ARWorldTrackingConfiguration()
        configuration.planeDetection = .horizontal
        
        // Run the view's session
        sceneView.session.run(configuration)
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        
        // Pause the view's session
        sceneView.session.pause()
    }
    
    func resetBox() {
        mode = .waitingForLocation
        box.resizeTo(min: .zero, max: .zero)
        currentAnchor = nil
        resultLabel.text = nil
        resultLabel.isHidden = true
    }
    
    // MARK: - Touch handling
    
    @objc dynamic func handlePan(_ gestureRecognizer: UIPanGestureRecognizer) {
        tapMode = false
        switch mode {
        case .waitingForLocation:
            findStartingLocation(gestureRecognizer)
        case .draggingInitialWidth:
            handleInitialWidthDrag(gestureRecognizer)
        case .draggingInitialLength:
            handleInitialLengthDrag(gestureRecognizer)
        case .waitingForFaceDrag:
            findFaceDragLocation(gestureRecognizer)
        case .draggingFace:
            handleFaceDrag(gestureRecognizer)
        }
    }
    
    @objc dynamic func handleDoubleTap(_ gestureRecognizer: UIPanGestureRecognizer) {
        resetBox()
        tapMode = false
        cloudMode = .none
    }
    
    // MARK: Twist-to-rotate gesture handling
    
    fileprivate var lastRotation = CGFloat(0)
    @objc dynamic func handleRotation(_ gestureRecognizer: UIRotationGestureRecognizer) {
        let currentRotation = gestureRecognizer.rotation
        switch gestureRecognizer.state {
        case .began:
            lastRotation = currentRotation
        case .changed:
            let rotationDelta = currentRotation - lastRotation
            lastRotation = currentRotation
            
            let rotation = SCNQuaternion(radians: -Float(rotationDelta), around: .axisY)
            let rotationPivot = box.pointInBounds(at: SCNVector3(x: 0.5, y: 0, z: 0.5))
            let pivotInWorld = box.convertPosition(rotationPivot, to: nil)
            box.rotate(by: rotation, aroundTarget: pivotInWorld)
        default:
            break
        }
    }
    
    // MARK: Drag Gesture handling
    
    func findStartingLocation(_ gestureRecognizer: UIPanGestureRecognizer) {
        switch gestureRecognizer.state {
        case .began, .changed:
            // Use real-world ARKit coordinates to determine where to start drawing
            let touchPos = gestureRecognizer.location(in: sceneView)
            
            let hit = realWorldHit(at: touchPos)
            if let startPos = hit.position, let plane = hit.planeAnchor {
                // Once the user hits a usable real-world plane, switch into line-dragging mode
                box.position = startPos
                currentAnchor = plane
                mode = .draggingInitialWidth
            }
        default:
            break
        }
    }
    
    // MARK: Tap Gesture handling
    
    @objc dynamic func handleTap(_ gestureRecognizer: UITapGestureRecognizer) {
        
        switch gestureRecognizer.state {
        case .ended:
            // Use real-world ARKit coordinates to determine where to start drawing
            let touchPos = gestureRecognizer.location(in: sceneView)
           
//            let hitTestResults = sceneView.hitTest(touchPos, types: .featurePoint)
//            guard !hitTestResults.isEmpty else { return  }
//
//            let firstResult = hitTestResults.first!
//
//            for point in pointCloud.currentFramePoints {
//
//                let resultPosition = firstResult.worldTransform.position4
//                let x = fabs(resultPosition.x - point.x) < 0.01
//                let y = fabs(resultPosition.y - point.y) < 0.01
//                let z = fabs(resultPosition.z - point.z) < 0.01
//
//                if true {
//                    let position = firstResult.worldTransform.position
////                    let hitPos = self.makeVertex()
////                    hitPos.isHidden = false
////                    hitPos.position = position
//
//                    switch cloudMode {
//                    case .none:
//                        cloudMode = .point
//
//                        box.position = position
//                        mode = .draggingInitialWidth
//                    case .point:
//                        cloudMode = .line
//
//                        let delta = box.position - position
//                        let distance = delta.length
//                        let angleInRadians = atan2(delta.z, delta.x)
//
//                        box.move(side: .right, to: distance)
//                        box.rotation = SCNVector4(x: 0, y: 1, z: 0, w: -(angleInRadians + Float.pi))
//                        mode = .draggingInitialLength
//
//                    case .line:
//                        cloudMode = .box
//
//                        // Check where the hit vector landed within the box's own coordinate system, which may be rotated.
//                        let locationInBox = box.convertPosition(position, from: nil)
//
//                        // Front side faces toward +z, back side toward -z
//                        if locationInBox.z < 0 {
//                            box.move(side: .front, to: 0)
//                            box.move(side: .back, to: locationInBox.z)
//                        } else {
//                            box.move(side: .front, to: locationInBox.z)
//                            box.move(side: .back, to: 0)
//                        }
//
//                    default:
//                        break
//                    }
//
//                    return
//                }
//            }
//
//
//            return
            let hit = realWorldHit(at: touchPos)
            if let startPos = hit.position, let plane = hit.planeAnchor {
                // Once the user hits a usable real-world plane, switch into line-dragging mode
                tapMode = true
                switch mode {
                case .waitingForLocation:
                    tapA.position = startPos
                    box.position = startPos
                    currentAnchor = plane
                    mode = .draggingInitialWidth
                    
                case .draggingInitialWidth:
                    
                    let delta = box.position - startPos
                    let distance = delta.length
                    let angleInRadians = atan2(delta.z, delta.x)
                    
                    box.move(side: .right, to: distance)
                    box.rotation = SCNVector4(x: 0, y: 1, z: 0, w: -(angleInRadians + Float.pi))
                    mode = .draggingInitialLength
                    
                default:
                    break
                }
    
                
            }
        default:
            break
        }
    }
    
	func handleInitialWidthDrag(_ gestureRecognizer: UIPanGestureRecognizer) {
		switch gestureRecognizer.state {
		case .changed:
			let touchPos = gestureRecognizer.location(in: sceneView)
			if let locationInWorld = scenekitHit(at: touchPos, within: hitTestPlane) {
				// This drags a line out that determines the box's width and its orientation:
				// The box's front will face 90 degrees clockwise out from the line being dragged.
				let delta = box.position - locationInWorld
				let distance = delta.length
				
				let angleInRadians = atan2(delta.z, delta.x)
				
				box.move(side: .right, to: distance)
				box.rotation = SCNVector4(x: 0, y: 1, z: 0, w: -(angleInRadians + Float.pi))
			}
		case .ended, .cancelled:
			if abs(box.boundingBox.max.x - box.boundingBox.min.x) >= box.minLabelDistanceThreshold {
				// If the box ended up with a usable width, switch to length-dragging mode.
				mode = .draggingInitialLength
			} else {
				// Otherwise, give up on this drag and start again.
				resetBox()
			}
		default:
			break
		}
	}
	
	func handleInitialLengthDrag(_ gestureRecognizer: UIPanGestureRecognizer) {
		switch gestureRecognizer.state {
		case .changed:
			let touchPos = gestureRecognizer.location(in: sceneView)
			if let locationInWorld = scenekitHit(at: touchPos, within: hitTestPlane) {
				// Check where the hit vector landed within the box's own coordinate system, which may be rotated.
				let locationInBox = box.convertPosition(locationInWorld, from: nil)
				
				// Front side faces toward +z, back side toward -z
				if locationInBox.z < 0 {
					box.move(side: .front, to: 0)
					box.move(side: .back, to: locationInBox.z)
				} else {
					box.move(side: .front, to: locationInBox.z)
					box.move(side: .back, to: 0)
				}
			}
		case .ended, .cancelled:
			// Once the box has a usable width and depth, switch to face-dragging mode.
			// Otherwise, stay in length-dragging mode.
			if (box.boundingBox.max.z - box.boundingBox.min.z) >= box.minLabelDistanceThreshold {
				mode = .waitingForFaceDrag
			}
		default:
			break
		}
	}
	
	func findFaceDragLocation(_ gestureRecognizer: UIPanGestureRecognizer) {
		switch gestureRecognizer.state {
		case .began, .changed:
			let touchPos = gestureRecognizer.location(in: sceneView)
			
			// Test if the user managed to hit a face of the box: if so, transition into dragging that face
			for (side, node) in box.faces {
				let hitResults = sceneView.hitTest(touchPos, options: [
					.rootNode: node,
					.firstFoundOnly: true,
				])
				
				if let result = hitResults.first {
					let coordinatesInBox = box.convertPosition(result.localCoordinates, from: result.node)
					mode = .draggingFace(side: side, dragStart: coordinatesInBox)
					return
				}
			}
		default:
			break
		}
	}
	
	func handleFaceDrag(_ gestureRecognizer: UIPanGestureRecognizer) {
		guard case let .draggingFace(side, _) = mode else {
			return
		}
		
		switch gestureRecognizer.state {
		case .changed:
			let touchPos = gestureRecognizer.location(in: sceneView)
			if let locationInWorld = scenekitHit(at: touchPos, within: hitTestPlane) {
				// Check where the hit vector landed within the box's own coordinate system, which may be rotated.
				let locationInBox = box.convertPosition(locationInWorld, from: nil)
				
				var distanceForAxis = locationInBox.value(for: side.axis)
				
				// Don't allow the box to be dragged inside-out: stop dragging the side at the point at which it meets its opposite side.
				switch side.edge {
				case .min:
					distanceForAxis = min(distanceForAxis, box.boundingBox.max.value(for: side.axis))
				case .max:
					distanceForAxis = max(distanceForAxis, box.boundingBox.min.value(for: side.axis))
				}
				
				box.move(side: side, to: distanceForAxis)
			}
		case .ended, .cancelled:
			mode = .waitingForFaceDrag
		default:
			break
		}
	}
	
	// MARK: - Hit-testing
	
	func scenekitHit(at screenPos: CGPoint, within rootNode: SCNNode) -> SCNVector3? {
		let hits = sceneView.hitTest(screenPos, options: [
			.boundingBoxOnly: true,
			.firstFoundOnly: true,
			.rootNode: rootNode,
			.ignoreChildNodes: true
		])
		
		return hits.first?.worldCoordinates
	}
	
	func realWorldHit(at screenPos: CGPoint) -> (position: SCNVector3?, planeAnchor: ARPlaneAnchor?, hitAPlane: Bool) {
		
		// -------------------------------------------------------------------------------
		// 1. Always do a hit test against exisiting plane anchors first.
		//    (If any such anchors exist & only within their extents.)
		
		let planeHitTestResults = sceneView.hitTest(screenPos, types: .existingPlaneUsingExtent)
		if let result = planeHitTestResults.first {
			
			let planeHitTestPosition = SCNVector3.positionFromTransform(result.worldTransform)
			let planeAnchor = result.anchor
			
			// Return immediately - this is the best possible outcome.
			return (planeHitTestPosition, planeAnchor as? ARPlaneAnchor, true)
		}
		
		// -------------------------------------------------------------------------------
		// 2. Collect more information about the environment by hit testing against
		//    the feature point cloud, but do not return the result yet.
		
		var featureHitTestPosition: SCNVector3?
		var highQualityFeatureHitTestResult = false
		
		let highQualityfeatureHitTestResults = sceneView.hitTestWithFeatures(screenPos, coneOpeningAngleInDegrees: 18, minDistance: 0.2, maxDistance: 2.0)
		
		if !highQualityfeatureHitTestResults.isEmpty {
			let result = highQualityfeatureHitTestResults[0]
			featureHitTestPosition = result.position
			highQualityFeatureHitTestResult = true
		}
		
		// -------------------------------------------------------------------------------
		// 4. If available, return the result of the hit test against high quality
		//    features if the hit tests against infinite planes were skipped or no
		//    infinite plane was hit.
		
		if highQualityFeatureHitTestResult {
			return (featureHitTestPosition, nil, false)
		}
		
		// -------------------------------------------------------------------------------
		// 5. As a last resort, perform a second, unfiltered hit test against features.
		//    If there are no features in the scene, the result returned here will be nil.
		
		let unfilteredFeatureHitTestResults = sceneView.hitTestWithFeatures(screenPos)
		if !unfilteredFeatureHitTestResults.isEmpty {
			let result = unfilteredFeatureHitTestResults[0]
			return (result.position, nil, false)
		}
		
		return (nil, nil, false)
	}
	
	
    // MARK: - ARSCNViewDelegate
	
	// Highlight detected planes in the view with a surface so we can see what the hell we're doing
    func renderer(_ renderer: SCNSceneRenderer, nodeFor anchor: ARAnchor) -> SCNNode? {
		guard let planeAnchor = anchor as? ARPlaneAnchor else {
			return nil
		}
		
		let plane = SCNBox(width: CGFloat(planeAnchor.extent.x),
		                   height: 0.0001,
		                   length: CGFloat(planeAnchor.extent.z), chamferRadius: 0)
		
		if let material = plane.firstMaterial {
			material.lightingModel = .constant
			material.diffuse.contents = UIColor.blue
			material.transparency = 0.1
			material.writesToDepthBuffer = false
		}
		
		let node = SCNNode(geometry: plane)
		node.categoryBitMask = RenderingCategory.planes.rawValue
		
        return node
    }
	
	func renderer(_ renderer: SCNSceneRenderer, didUpdate node: SCNNode, for anchor: ARAnchor) {
		guard let planeAnchor = anchor as? ARPlaneAnchor, let plane = node.geometry as? SCNBox else {
			return
		}
		
		plane.width = CGFloat(planeAnchor.extent.x)
		plane.length = CGFloat(planeAnchor.extent.z)
		
		// If this anchor is the one the box is positioned relative to, then update the box to match any corrections to the plane's observed position.
		if plane == currentAnchor {
			let oldPos = node.position
			let newPos = SCNVector3.positionFromTransform(planeAnchor.transform)
			let delta = newPos - oldPos
			box.position += delta
		}
		
		node.transform = SCNMatrix4(planeAnchor.transform)
		node.pivot = SCNMatrix4(translationByX: -planeAnchor.center.x, y: -planeAnchor.center.y, z: -planeAnchor.center.z)
	}
	
    func session(_ session: ARSession, didFailWithError error: Error) {
        // Present an error message to the user
        
    }
    
    func sessionWasInterrupted(_ session: ARSession) {
        // Inform the user that the session has been interrupted, for example, by presenting an overlay
        
    }
    
    func sessionInterruptionEnded(_ session: ARSession) {
        // Reset tracking and/or remove existing anchors if consistent tracking is required
        
    }
    
    fileprivate func makeNode(with geometry: SCNGeometry) -> SCNNode {
        for material in geometry.materials {
            material.lightingModel = .constant
            material.diffuse.contents = UIColor.white
            material.isDoubleSided = false
        }
        
        let node = SCNNode(geometry: geometry)
        node.isHidden = true
        sceneView.scene.rootNode.addChildNode(node)
        return node
    }
    
    fileprivate func makeVertex() -> SCNNode {
        let ball = SCNSphere(radius: CGFloat(0.005))
        return makeNode(with: ball)
    }
    
    //MARK: Helper
    fileprivate func updateResultData(_ length: Float, _ width: Float, _ height: Float) {
        guard length > 0, width > 0, height > 0 else  {
            resultLabel.isHidden = true
            return
        }
        
        let formatLength = lengthFormatter.string(for: NSNumber(value: length))!
        let formatWidth = lengthFormatter.string(for: NSNumber(value: width))!
        let formatHeight = lengthFormatter.string(for: NSNumber(value: height))!

        let box = length * width * height
        
        let formatBox = lengthFormatter.string(for: NSNumber(value: box))!

        let result = "长: \(formatLength)cm\n宽: \(formatWidth)cm\n高: \(formatHeight)cm\n体积: \(box)m³"
        resultLabel.text = result
        resultLabel.isHidden = false
    }
}

extension ViewController: ARSessionDelegate {
    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        guard let frame = sceneView.session.currentFrame else { return }
        let now = CACurrentMediaTime()
        if now - timeOfLastReferenceObjectCreation > 0.1 {
            timeOfLastReferenceObjectCreation = now
            
//            let v = SIMD4<Float>(0.01)
//            let transform = simd_float4x4(v, v, v, v)
//            let extent = SIMD3<Float>(0.1, 0.1, 0.1)
//            sceneView.session.createReferenceObject(transform: box.simdWorldTransform, center: SIMD3<Float>(), extent: extent) { object, error in
//                if let refrenceObject = object {
//                    self.pointCloud.update(with: refrenceObject.rawFeaturePoints)
//                }
//            }
            
            if let currentPoints = frame.rawFeaturePoints {
                pointCloud.update(with: currentPoints)
            }
        }
        
        pointCloud.updateOnEveryFrame()
    }
    
    func renderer(_ renderer: SCNSceneRenderer, didAdd node: SCNNode, for anchor: ARAnchor) {
        print("did add")
    }
    
    func renderer(_ renderer: SCNSceneRenderer, willUpdate node: SCNNode, for anchor: ARAnchor) {
        print("will update")
    }
    
    func renderer(_ renderer: SCNSceneRenderer, didRemove node: SCNNode, for anchor: ARAnchor) {
        print("did remove")
    }
    
    func renderer(_ renderer: SCNSceneRenderer, didRenderScene scene: SCNScene, atTime time: TimeInterval) {
//        print("did render scene")
    }
    
    func renderer(_ renderer: SCNSceneRenderer, didApplyAnimationsAtTime time: TimeInterval) {
//        print("did apply animations")
    }
    
    func renderer(_ renderer: SCNSceneRenderer, didSimulatePhysicsAtTime time: TimeInterval) {
//        print("did simulate physics")
    }
    
    func renderer(_ renderer: SCNSceneRenderer, didApplyConstraintsAtTime time: TimeInterval) {
//        print("did apply constraints")
    }
    
    func renderer(_ renderer: SCNSceneRenderer, willRenderScene scene: SCNScene, atTime time: TimeInterval) {
//        print("will render scene")
    }
}

extension float4x4 {
    var position4: SIMD3<Float> {
        return columns.3.xyz
    }
}

extension SIMD4 where Scalar == Float {
    var xyz: SIMD3<Float> {
        return SIMD3<Float>(x, y, z)
    }

    init(_ xyz: SIMD3<Float>, _ w: Float) {
        self.init(xyz.x, xyz.y, xyz.z, w)
    }
}

extension ViewController: BoxUpdate {
    func update(_ length: Float, _ width: Float, _ height: Float) {
        self.updateResultData(length, width, height)
    }
}
