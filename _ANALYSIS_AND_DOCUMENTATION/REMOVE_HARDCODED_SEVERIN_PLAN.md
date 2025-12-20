# Improved Plan: Remove Hardcoded "Severin" References

This plan replaces hardcoded references with a dynamic, architecture-compliant approach using the existing Person Management System.

## Architecture Improvements
1.  **Promote PersonRegistry:** Move `tests/face_recognition/person_registry.py` to a new ROS 2 package `r2d2_common`. This makes it a first-class citizen accessible to all nodes and launch files.
2.  **Persistent Storage:** Ensure `PersonRegistry` uses a fixed persistent path (`~/dev/r2d2/data/persons.db`) instead of a relative path, as installed ROS packages reside in non-persistent locations.
3.  **Dynamic Resolution:** Nodes and launch files will query `PersonRegistry` to find the default person and their model paths.

## Implementation Steps

### Phase 1: Common Infrastructure (The Foundation)
1.  **Create `r2d2_common` Package:**
    *   Create standard ROS 2 python package structure in `ros2_ws/src/r2d2_common`.
    *   Migrate `person_registry.py` to this package.
    *   **CRITICAL FIX:** Modify `person_registry.py` to default to `~/dev/r2d2/data/persons.db` if no path provided, ensuring data persistence.
    *   Add `r2d2_common` dependency to other packages.

2.  **Initialize Registry:**
    *   Run `auto_migrate()` using the new package to populate the database from existing model files on disk (`severin_lbph.xml`, etc.).
    *   This ensures "severin" (and any others) are registered immediately.

### Phase 2: Node Updates (The Logic)
3.  **Update `audio_notification_node`:**
    *   Import `PersonRegistry` from `r2d2_common`.
    *   Remove `default='severin'`.
    *   Logic: If `target_person` param is generic ('target_person'), query registry for the first available person.

4.  **Update `image_listener` (Perception):**
    *   Import `PersonRegistry` from `r2d2_common`.
    *   Remove hardcoded model paths in `declare_parameter`.
    *   Logic: If model paths are not provided, query registry for the default person's models.

### Phase 3: Configuration Updates (The Glue)
5.  **Update Launch Files:**
    *   `r2d2_camera_perception.launch.py`: Remove hardcoded default paths. Use `PersonRegistry` to find models dynamically if arguments aren't provided.
    *   `audio_notification.launch.py`: Allow dynamic target person resolution.

6.  **Update Systemd Services:**
    *   `r2d2-camera-perception.service`: Remove `gesture_recognition_model_path:=...` and `target_person_name:=...` arguments. Let the launch file/node handle it.
    *   This makes the service generic for ANY user.

### Phase 4: Cleanup (The Polish)
7.  **Startup Scripts & Docs:**
    *   Update `start_audio_notification.sh` comments.
    *   Update documentation examples to use generic names (e.g., "alice" or "user") instead of "severin", or explicitly state "severin" is just an example.

## Verification
*   **Test:** Reboot system.
*   **Success Criteria:**
    *   System starts without errors.
    *   "Severin" is recognized (because `auto_migrate` found the model).
    *   Audio alerts work.
    *   Gestures work.
    *   No "severin" string exists in the active code logic (only in the database/data files).

