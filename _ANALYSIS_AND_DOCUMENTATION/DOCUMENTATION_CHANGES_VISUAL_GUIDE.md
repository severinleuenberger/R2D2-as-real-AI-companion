# Documentation Changes - Visual Guide
## Before/After Comparison for Remove Hardcoded Severin

**Date:** December 20, 2025  
**Purpose:** Show specific documentation changes with visual comparisons

---

## 1. Architecture Overview (001_ARCHITECTURE_OVERVIEW.md)

### Change 1: Hardware Constants Table (Section 1.2)

**BEFORE:**
```markdown
| **Face Recognition Model Path** | `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml` | File system | Default model location |
```

**AFTER:**
```markdown
| **Person Registry Database** | `~/dev/r2d2/data/persons.db` | File system | Central person entity database |
| **Face Recognition Models** | `~/dev/r2d2/data/face_recognition/models/{person}_lbph.xml` | File system | Per-person face models (auto-resolved) |
| **Gesture Recognition Models** | `~/dev/r2d2/data/gesture_recognition/models/{person}_gesture_classifier.pkl` | File system | Per-person gesture models (auto-resolved) |
```

**Impact:** Now shows the registry database as the source of truth, with generic model path patterns.

---

### Change 2: Launch Examples (Section 6.1)

**BEFORE:**
```markdown
# With gesture recognition enabled
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  enable_gesture_recognition:=true \
  gesture_recognition_model_path:=/home/severin/dev/r2d2/data/gesture_recognition/models/severin_gesture_classifier.pkl
```

**AFTER:**
```markdown
# With gesture recognition enabled (model paths auto-resolved from PersonRegistry)
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  enable_gesture_recognition:=true
```

**Impact:** Simpler launch commands - no need to specify model paths.

---

### Change 3: New Section 7.5 - Person Registry Integration

**ADDED NEW SECTION:**

```markdown
### 7.5 Person Registry Integration (Dynamic Multi-User Support)

The R2D2 system uses a centralized Person Registry to eliminate hardcoded person names 
and model paths, enabling true multi-user support without code changes.

**Key Architecture:**
- **Database:** ~/dev/r2d2/data/persons.db (persistent SQLite)
- **Common Package:** r2d2_common provides PersonConfig for ROS 2 nodes
- **Auto-Resolution:** When parameters use 'auto' or 'target_person', the system 
  queries the PersonRegistry

**How It Works:**

Training System → PersonRegistry Database (persons.db)
                       ↓
                  PersonConfig (r2d2_common)
                       ↓
    ┌──────────────────┼──────────────────┐
    ↓                  ↓                  ↓
image_listener    audio_notification  Launch Files
(face + gesture)  (target person)     (auto-resolve)

**Benefits:**
- No hardcoded person names in code (only in database)
- Adding new users requires only training, not code changes
- Multi-user support: any trained person is automatically authorized
- Model paths automatically resolved at runtime
```

**Impact:** Users now understand the PersonRegistry architecture.

---

## 2. System Integration Reference (007_SYSTEM_INTEGRATION_REFERENCE.md)

### Change 1: Multi-User Authorization Note

**BEFORE:**
```markdown
No hardcoded names are required. When switching users, only the face model path
needs to change; no code changes are needed.
```

**AFTER:**
```markdown
No hardcoded names are required. The system uses PersonRegistry (r2d2_common)
to auto-resolve model paths at runtime. Adding new users requires only training - 
no code or configuration changes needed.
```

**Impact:** Clearer explanation of the auto-resolution mechanism.

---

### Change 2: image_listener Configuration

**BEFORE:**
```markdown
**image_listener:**
- `target_person_name`: "severin" (must match training)
- `face_presence_threshold`: 2.0 (seconds to confirm presence)
```

**AFTER:**
```markdown
**image_listener:**
- `target_person_name`: "target_person" (auto-resolved via PersonRegistry)
- `face_recognition_model_path`: "auto" (auto-resolved from PersonRegistry)
- `gesture_model_path`: "auto" (auto-resolved from PersonRegistry)
- `face_presence_threshold`: 2.0 (seconds to confirm presence)
```

**Impact:** Shows the new auto-resolution parameters.

---

### Change 3: Service Parameters

**BEFORE:**
```markdown
**Parameters:**
- `target_person_name:=severin` (configured in service)
```

**AFTER:**
```markdown
**Parameters:**
- Model paths auto-resolved from PersonRegistry (no hardcoded parameters)
```

**Impact:** Service files no longer need person-specific parameters.

---

## 3. Person Recognition Reference (100_PERSON_RECOGNITION_REFERENCE.md)

### Change 1: Launch Parameters Table

**BEFORE:**
```markdown
| `face_recognition_model_path` | `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml` | string | Path to trained model |
```

**AFTER:**
```markdown
| `face_recognition_model_path` | `auto` | string | Path to trained model (auto-resolved from PersonRegistry, or provide explicit path) |
```

**Impact:** Default is now 'auto' which triggers PersonRegistry lookup.

---

### Change 2: File Locations

**BEFORE:**
```markdown
**Training Data:**
- Images: `~/dev/r2d2/data/face_recognition/severin/`
- Model: `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml`
```

**AFTER:**
```markdown
**Training Data:**
- Images: `~/dev/r2d2/data/face_recognition/{person}/`
- Models: `~/dev/r2d2/data/face_recognition/models/{person}_lbph.xml`
- Registry: `~/dev/r2d2/data/persons.db` (central person database)
```

**Impact:** Generic {person} placeholder shows multi-user capability.

---

## 4. Person Management System (250_PERSON_MANAGEMENT_SYSTEM_REFERENCE.md)

### Change 1: Database Location

**BEFORE:**
```markdown
- **Database:** `tests/face_recognition/data/persons.db`
```

**AFTER:**
```markdown
- **Database:** `~/dev/r2d2/data/persons.db` (persistent location)
```

**Impact:** Shows the new absolute persistent path.

---

### Change 2: NEW - Enhanced Architecture Diagram

**ADDED:**

```mermaid
flowchart TB
    subgraph TrainingLayer [Training Layer]
        TrainManager[train_manager.py<br/>Face + Gesture Training]
    end
    
    subgraph DataLayer [Data Layer - Persistent Storage]
        PersonsDB[(persons.db<br/>~/dev/r2d2/data/persons.db)]
        FaceModels[Face Models<br/>{person}_lbph.xml]
        GestureModels[Gesture Models<br/>{person}_gesture_classifier.pkl]
    end
    
    subgraph CommonLayer [r2d2_common Package - Abstraction]
        PersonRegistry[PersonRegistry<br/>Database API]
        PersonConfig[PersonConfig<br/>ROS 2 Helper]
    end
    
    subgraph ROS2Layer [ROS 2 Nodes - Runtime]
        ImageListener[image_listener<br/>Perception Node]
        AudioNotif[audio_notification_node<br/>State Machine]
    end
    
    TrainManager -->|auto_migrate| PersonRegistry
    PersonRegistry --> PersonsDB
    PersonsDB -.links to.-> FaceModels
    PersonsDB -.links to.-> GestureModels
    
    PersonRegistry --> PersonConfig
    PersonConfig --> ImageListener
    PersonConfig --> AudioNotif
```

**Impact:** Visual representation of the complete system architecture.

---

### Change 3: NEW - r2d2_common Integration Section

**ADDED:**

```markdown
### For ROS 2 Nodes via r2d2_common

**PersonConfig Integration:**

Location: `ros2_ws/src/r2d2_common/r2d2_common/person_config.py`

**Integration Flow:**

[Mermaid diagram showing ROS2Nodes → PersonConfig → PersonRegistry → DB]

**Usage in Nodes:**
```python
from r2d2_common.person_config import PersonConfig

# Resolve person name
self.target_person = PersonConfig.get_person_name('target_person')

# Resolve model paths
face_model_path = PersonConfig.get_face_model_path('target_person')
```

**Key Methods:**
- PersonConfig.get_person_name() → Resolved person name
- PersonConfig.get_face_model_path() → Face model path
- PersonConfig.get_gesture_model_path() → Gesture model path
```

**Impact:** Developers now have clear examples of how to use PersonConfig in their nodes.

---

### Change 4: Service Configuration

**BEFORE:**
```markdown
**Service Configuration:**
# Model path in /etc/systemd/system/r2d2-camera-perception.service
gesture_recognition_model_path:=/home/severin/dev/r2d2/data/gesture_recognition/models/severin_gesture_classifier.pkl

**Update After New Training:**
sudo nano /etc/systemd/system/r2d2-camera-perception.service
# Change gesture_recognition_model_path to new person's model
```

**AFTER:**
```markdown
**Service Configuration:**
# /etc/systemd/system/r2d2-camera-perception.service
# No model paths needed - auto-resolved from PersonRegistry!
ExecStart=... ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  enable_gesture_recognition:=true

**Update After New Training:**
# Just train the new person - no service file changes needed!
cd ~/dev/r2d2/tests/face_recognition
python3 train_manager.py

# System auto-discovers new person from PersonRegistry on next restart
sudo systemctl restart r2d2-camera-perception.service
```

**Impact:** Dramatically simplified deployment process.

---

## 5. Quick Start Guide (008_SYSTEM_INTEGRATION_QUICK_START.md)

### Change: Person ID Expectations

**BEFORE:**
```markdown
timeout 3 ros2 topic echo /r2d2/perception/person_id --once
# EXPECT: "severin" (when in front of camera)
```

**AFTER:**
```markdown
timeout 3 ros2 topic echo /r2d2/perception/person_id --once
# EXPECT: Registered person name from PersonRegistry (e.g., "alice", "bob")
# The system auto-resolves via PersonConfig
```

**Impact:** Users understand output is dynamic, not hardcoded.

---

## 6. Troubleshooting Guide (009_SYSTEM_INTEGRATION_TROUBLESHOOTING.md)

### Change: NEW - PersonRegistry Diagnostics Section

**ADDED:**

```markdown
## PersonRegistry Troubleshooting

### Issue: "Person Not Resolved" / "target_person" Showing Instead of Actual Name

**Symptom:** System shows "target_person" instead of actual person name

**Root Cause:** PersonRegistry is empty or database not found

**Diagnosis:**
```bash
# Check if persons are registered
python3 -c "from person_registry import PersonRegistry; r = PersonRegistry(); print(r.list_persons())"

# Check database exists
ls -la ~/dev/r2d2/data/persons.db
```

**Solution:**
```bash
# Run auto-migrate
python3 -c "from person_registry import PersonRegistry; r = PersonRegistry(); print(r.auto_migrate())"
```

**Expected:** At least one person should be listed with face/gesture model paths.
```

**Impact:** Users can now diagnose and fix PersonRegistry issues independently.

---

## 7. Installation Guides Updates

### 101_PERSON_RECOGNITION_INSTALLATION.md

**BEFORE:**
```markdown
Model saved to: ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml

ls -lh ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml
```

**AFTER:**
```markdown
Model saved to: ~/dev/r2d2/data/face_recognition/models/{person}_lbph.xml
Person registered in PersonRegistry

# Verify
ls -lh ~/dev/r2d2/data/face_recognition/models/*.xml
python3 -c "from person_registry import PersonRegistry; r = PersonRegistry(); print(r.list_persons())"
```

**Impact:** Verification now includes PersonRegistry check.

---

### 102_PERSON_RECOGNITION_QUICK_START.md

**BEFORE:**
```markdown
- Model: `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml`
```

**AFTER:**
```markdown
- Models: `~/dev/r2d2/data/face_recognition/models/{person}_lbph.xml`
- Registry: `~/dev/r2d2/data/persons.db` (auto-resolution database)
```

**Impact:** Shows the registry as part of the system architecture.

---

### 303_GESTURE_TRAINING_GUIDE.md

**BEFORE:**
```markdown
sudo nano /etc/systemd/system/r2d2-camera-perception.service
# Verify gesture_recognition_model_path points to your trained model
# Default: /home/severin/dev/r2d2/data/gesture_recognition/models/severin_gesture_classifier.pkl
```

**AFTER:**
```markdown
# No service file changes needed - models auto-resolved from PersonRegistry!
cat /etc/systemd/system/r2d2-camera-perception.service | grep "launch.py"
# Should NOT contain hardcoded gesture_recognition_model_path
```

**Impact:** Users understand no manual configuration is needed.

---

## Key Diagrams Enhanced

### Diagram 1: Person Registry Architecture (250_PERSON_MANAGEMENT_SYSTEM_REFERENCE.md)

**New Mermaid Flowchart Added:**

Shows the complete flow from training through data storage, abstraction layer (r2d2_common), to runtime nodes. This diagram was previously just ASCII art - now it's an interactive Mermaid diagram.

**Key Additions:**
- Shows all 4 layers clearly
- Shows data flow with arrows
- Shows abstraction (PersonRegistry → PersonConfig)
- Shows runtime usage (nodes)

---

### Diagram 2: System Integration Flow (007_SYSTEM_INTEGRATION_REFERENCE.md)

**Enhanced State Machine Diagram:**

Added note about "DYNAMIC PERSON: Any trained person triggers RED" to clarify that the system is not hardcoded to a specific person.

---

### Diagram 3: Before/After Architecture (IMPLEMENTATION_COMPLETE.md)

**New Diagram Created:**

```mermaid
flowchart TB
    subgraph Training [Training Time]
        UserTrains[User trains face + gestures]
        TrainManager[train_manager.py]
        AutoRegister[auto_migrate]
    end
    
    subgraph Storage [Persistent Storage]
        PersonsDB[(persons.db)]
        FaceModels[{person}_lbph.xml]
        GestureModels[{person}_gesture_classifier.pkl]
    end
    
    subgraph Runtime [Runtime Resolution]
        PersonRegistry[PersonRegistry]
        PersonConfig[PersonConfig r2d2_common]
        Nodes[ROS 2 Nodes auto-resolve]
    end
    
    UserTrains --> TrainManager
    TrainManager --> AutoRegister
    AutoRegister --> PersonRegistry
    PersonRegistry --> PersonsDB
    PersonsDB -.links.-> FaceModels
    PersonsDB -.links.-> GestureModels
    
    PersonRegistry --> PersonConfig
    PersonConfig --> Nodes
```

**Impact:** Complete visual representation of the new architecture.

---

## Documentation Philosophy Changes

### Old Philosophy
- **Examples:** Use "severin" everywhere as the example person
- **Paths:** Show specific hardcoded paths in examples
- **Configuration:** Show exact service file content with hardcoded values

### New Philosophy
- **Examples:** Use generic placeholders like {person}, "alice", "bob"
- **Paths:** Show generic patterns and auto-resolution
- **Configuration:** Show minimal configuration (auto-resolution by default)
- **Explanation:** Always explain that values are resolved from PersonRegistry

---

## Testing the Documentation Changes

### Verification Script

```bash
#!/bin/bash
# Verify documentation no longer has hardcoded severin in critical sections

echo "Checking for hardcoded 'severin' in code examples..."

# Check documentation files (exclude path examples and file locations)
for doc in 001_ARCHITECTURE_OVERVIEW.md 007_SYSTEM_INTEGRATION_REFERENCE.md \
           100_PERSON_RECOGNITION_REFERENCE.md 250_PERSON_MANAGEMENT_SYSTEM_REFERENCE.md; do
    
    matches=$(grep -i "target_person.*severin\|default.*severin" ~/dev/r2d2/$doc 2>/dev/null | wc -l)
    
    if [ $matches -gt 0 ]; then
        echo "⚠️  $doc: $matches matches (review needed)"
    else
        echo "✓ $doc: clean"
    fi
done

echo ""
echo "Note: File path examples like '/home/severin/...' are OK (system-specific paths)"
echo "      Only checking for hardcoded person names in configuration/code examples"
```

---

## User-Facing Changes Summary

| Aspect | Before | After |
|--------|--------|-------|
| **Adding User** | Train + Edit 3 files | Train only |
| **Service Files** | Person-specific params | Generic (no params) |
| **Launch Commands** | Model paths required | Model paths auto-resolved |
| **Documentation Examples** | "severin" everywhere | Generic {person} or "alice" |
| **Default Behavior** | Hardcoded to severin | First registered person |
| **Multi-User** | Manual switching | Automatic recognition |

---

## Next Documentation Tasks (Future)

While the current documentation is now updated, consider these future enhancements:

1. **User Guide:** Create "HOW_TO_ADD_USERS.md" with step-by-step multi-user setup
2. **Architecture Diagram:** Add PersonRegistry to main system architecture diagram
3. **FAQ Section:** Add "How do I switch between users?" to each document
4. **Migration Guide:** Create upgrade guide for existing deployments

---

**Document Version:** 1.0  
**Date:** December 20, 2025  
**Purpose:** Visual comparison of documentation changes  
**Status:** Reference for review and validation

