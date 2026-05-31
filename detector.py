from ultralytics import YOLO

class TargetData:
    """A simple data container"""
    def __init__(self, found=False, current_area=0.0, bbox=None, xywhn=None):
        self.found = found
        self.current_area = current_area
        self.bbox = bbox
        self.xywhn = xywhn

    
class TargetDetector:
    def __init__(self, model_name="yolo26n.pt", target_class_name="car"):
        self.model = YOLO(model_name)
        
        # YOLO stores class names in a dictionary like {0: 'person', 2: 'car'}
        # We reverse this so we can look up the ID using your string ("car" -> 2)
        self.class_map = {name.lower(): class_id for class_id, name in self.model.names.items()}
        
        target_lower = target_class_name.lower()
        if target_lower not in self.class_map:
            raise ValueError(f"Target '{target_class_name}' is not recognized by this YOLO model.")
        
        self.target_class_id = self.class_map[target_lower]
        print(f"Detector initialized. Tracking '{target_class_name}' (Class ID: {self.target_class_id})")

    def find_target(self, frame):
        """Processes a frame and returns a TargetData object."""

        # Run inference quietly
        results = self.model(frame, verbose=False)
        boxes = results[0].boxes

        # Instantly filter for our target ID without slow python loops
        target_indices = (boxes.cls == self.target_class_id).nonzero().flatten()

        if len(target_indices) > 0:
            # Grab the very first matching target found
            first_idx = target_indices[0]
            
            # Extract normalized stats for logic
            xywhn = boxes.xywhn[first_idx].cpu().numpy()
            area = xywhn[2] * xywhn[3]
            
            # Extract raw pixels for drawing on the screen
            bbox = boxes.xyxy[first_idx].cpu().numpy()
            
            return TargetData(found=True, current_area=area, bbox=bbox, xywhn=xywhn)
            
        return TargetData(found=False)
    