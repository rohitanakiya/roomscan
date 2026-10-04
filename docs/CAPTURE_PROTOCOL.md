# Capture protocol (Route 2: stock apps) — one page

Follow this page literally. Total time: about 1 minute per room plus 2 minutes setup.

## 0. Install (once, < 2 minutes)
- **LiDAR tier** (iPhone 12 Pro or newer *Pro*/*Pro Max*, or iPad Pro with LiDAR): install **Stray Scanner** (free, App Store, by Stray Robots). Open it once and allow camera access.
- **Video and photo tiers** (any iPhone 15 or newer): nothing to install — use the built-in **Camera** app.

## 1. Prepare the space (1 minute)
1. Turn **all lights on**. Open curtains and blinds unless direct sun falls on the floor.
2. **Open every interior door fully** (leaf flat against its wall). Close nothing you want measured.
3. Keep people and pets out of the camera view while you capture.

## 2. LiDAR capture (Stray Scanner)
1. Press record standing in the **entrance**. Hold the phone **upright (portrait)**, at chest height.
2. Walk at **half normal speed**. Keep about **1 m from walls**. Never turn faster than a slow head turn.
3. In **every room**: walk its perimeter once. While walking, sweep the camera **up to the ceiling line and down to the floor line** on every wall (slow vertical "S"). Point at **each ceiling corner** for a second — ceiling height comes from this.
4. At **every doorway**: stop **in the doorway for 2 seconds**, look into the room you are leaving, then into the next room.
5. **Finish where you started** and look around the first room again for 5 seconds (this lets the software remove drift).
6. Stop recording. **Do not** point at mirrors for more than a second; do not cover the camera; do not walk through glass doors.
7. Hand-off: in Stray Scanner, open the recording → **Export → "Export all"** → AirDrop/Files to the computer. You get a folder containing `rgb.mp4`, `depth/`, `confidence/`, `odometry.csv`, `camera_matrix.csv`.

## 3. Video capture (Camera app)
1. Camera → **Video**, 1080p or 4K, 30 fps. Phone **upright (portrait)**.
2. Walk exactly the route of §2 (same sweeps, same doorway pauses, finish where you started).
3. Hand-off: AirDrop/cable the `.mov`/`.mp4` file to the computer **as the original** (AirDrop → Options → "All Photos Data" on).

## 4. Photo capture (Camera app) — 2 to 8 photos per room
In every room, phone **upright**, standing still for each shot:
1. Stand in **each corner** you can reach and photograph the **opposite corner**, showing floor and ceiling lines (up to 4 photos).
2. Stand **in each doorway** of the room and take **one photo into the next room** — this is how rooms are joined, never skip it.
3. Make a folder per room on the computer (any name, e.g. `kitchen`) and put that room's photos in it. All room folders go in one parent folder.

## 5. Run (one command per capture)
```
python -m roomscan <stray_export_folder>        # LiDAR
python -m roomscan <video_file.mov>             # video
python -m roomscan <parent_folder_of_room_folders>   # photos
```
Results: `out/<name>/result.json` and `out/<name>/plan.png`.

## What to avoid (and what happens if you don't)
| Avoid | Consequence |
|---|---|
| Fast turns / running | gaps in the walk; rooms split or misplaced |
| Skipping the doorway pause / doorway photo | room not joined to its neighbour (photo tier: placed by inference, flagged) |
| Never looking at the ceiling | ceiling height reported as *not observed* with a wide prior interval |
| Mirrors and glass held in view | phantom space behind the mirror; flagged in `warnings` |
| Low light | LiDAR is unaffected; video/photo depth degrades and intervals widen |
