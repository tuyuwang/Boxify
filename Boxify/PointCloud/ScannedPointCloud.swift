/*
See LICENSE folder for this sample’s licensing information.

Abstract:
A visualization of the 3D point cloud data during object scanning.
*/

import Foundation
import ARKit
import SceneKit

class ScannedPointCloud: SCNNode, PointCloud {
    
    private var pointNode = SCNNode()
    private var preliminaryPointsNode = SCNNode()
    
    // The latest known set of points inside the reference object.
    private var referenceObjectPoints: [SIMD3<Float>] = []
    
    // The current frame's set of points inside the reference object.
    public var currentFramePoints: [SIMD3<Float>] = []
    
    // The set of currently rendered points, in world coordinates.
    // Note: We render them in world coordinates instead of local coordinates to
    //       prevent rendering issues with points jittering e.g. when the
    //       bounding box is rotated.
    private var renderedPoints: [SIMD3<Float>] = []
    
    // The set of points from the current frame, in world coordinates.
    // Note: These are preliminary since not all of them might be added
    //       to the reference object.
    private var renderedPreliminaryPoints: [SIMD3<Float>] = []
    
    private var boundingBox: BoundingBox?
    
    private var pointCloudItems: [PointCloudItem] = []
    
    struct PointCloudItem {
        var identifier: UInt64
        var point: vector_float3
        
        init(pointCloud: ARPointCloud, index: Int) {
            self.identifier = pointCloud.identifiers[index]
            self.point = pointCloud.points[index]
        }
        
        
    }
    
    override init() {
        super.init()
        
        addChildNode(pointNode)
        addChildNode(preliminaryPointsNode)
        
//        NotificationCenter.default.addObserver(self,
//                                               selector: #selector(self.scanningStateChanged(_:)),
//                                               name: Scan.stateChangedNotification,
//                                               object: nil)
        NotificationCenter.default.addObserver(self,
                                               selector: #selector(self.boundingBoxPositionOrExtentChanged(_:)),
                                               name: BoundingBox.extentChangedNotification,
                                               object: nil)
        NotificationCenter.default.addObserver(self,
                                               selector: #selector(self.boundingBoxPositionOrExtentChanged(_:)),
                                               name: BoundingBox.positionChangedNotification,
                                               object: nil)
        NotificationCenter.default.addObserver(self,
                                               selector: #selector(self.scannedObjectPositionChanged(_:)),
                                               name: BoundingBox.positionChangedNotification,
                                               object: nil)
    }
    
    @objc
    func boundingBoxPositionOrExtentChanged(_ notification: Notification) {
        guard let boundingBox = notification.object as? BoundingBox else { return }
        updateBoundingBox(boundingBox)
    }
    
    @objc
    func scannedObjectPositionChanged(_ notification: Notification) {
        guard notification.object is BoundingBox else { return }
        updateBoundingBox(boundingBox)
    }
    
    func updateBoundingBox(_ boundingBox: BoundingBox?) {
        self.boundingBox = boundingBox
    }
    
    func update(with pointCloud: ARPointCloud, localFor boundingBox: BoundingBox) {
        // Convert the points to world coordinates because we display them
        // in world coordinates.
        var pointsInWorld: [SIMD3<Float>] = []
        for point in pointCloud.points {
            pointsInWorld.append(boundingBox.simdConvertPosition(point, to: nil))
        }
        
        print(pointCloud.identifiers)
        self.referenceObjectPoints = pointsInWorld
    }
    
    func update(with pointCloud: ARPointCloud) {

//        for index in 0...pointCloud.identifiers.count-1 {
//            let item = PointCloudItem(pointCloud: pointCloud, index: index)
//
//            var isNoExits = true
//
//            for i in pointCloudItems {
//                if (i.identifier == item.identifier) {
//                    isNoExits = false
//                }
//            }
//
//            if isNoExits {
//                pointCloudItems.append(item)
//            }
//        }
        
        self.currentFramePoints = pointCloud.points
    }
    
    func updateOnEveryFrame() {
        guard !self.isHidden else { return }
        guard !pointCloudItems.isEmpty else {
            self.pointNode.geometry = nil
            self.preliminaryPointsNode.geometry = nil
            return
        }
        
//        renderedPoints = []
//        renderedPreliminaryPoints = []
        
        // Abort if the bounding box has no extent yet
//        guard boundingBox.extent.x > 0 else { return }
        
        // Check which of the reference object's points and current frame's points are within the bounding box.
        // Note: The creation of the latest ARReferenceObject happens at a lower frequency
        //       than rendering and updates of the bounding box, so some of the points
        //       may no longer be inside of the box.
//        renderedPoints = referenceObjectPoints.filter { boundingBox.contains($0) }
//        renderedPreliminaryPoints = currentFramePoints.filter { boundingBox.contains($0) }
        
//        self.pointNode.geometry = createVisualization(for: renderedPoints, color: .appYellow, size: 12)
        
        var currentPoints: [SIMD3<Float>] = []
        for item in pointCloudItems {
            currentPoints.append(item.point)
        }
        
        self.preliminaryPointsNode.geometry = createVisualization(for: currentFramePoints, color: .appLightYellow, size: 12)
    }
    
    var count: Int {
        return renderedPoints.count
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
}
