# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.0.0]
### MIGRATION
- Merged the UPM version (this) with the Asset Store version. This now replaces the Asset Store version.
- GUIDs of scripts have been kept the same, meaning that for users of the Asset Store version, you simply need to delete all files of that Asset and re-import this UPM version.

### Changed
- Inserting control points no longer changes the shape of the existing segment.
- Certain pieces of code have been re-implemented using Burst, which results in 20x speed ups in some cases. One example is mesh generation, resulting in a smoother editing experience.

## [1.0.3]
### Changed
- Fixed mesh generation of meshes with many submeshes

## [1.0.2]
### Changed
- Fixed creation of settings directory

## [1.0.1]
### Changed
- Fixed Alignment of the ends of roads

## [1.0.0]
### Added
- New Intersection Button

## [0.4.0]
### Removed
- nunit Tests

### Changed
- The Script files didn't actually update between 0.2.4 and 0.3.0 even though I uploaded the package.

## [0.3.0]
### Changed
- Replaced roll-angle based road orientation with normals
- Normal interpolation along bezier updated to work with vertical roads as well
- Refactor Bezier class

## [0.2.4]
### Added
- Usage Documentation
- API Documentation
- Road Mesh clipping
- Custom Mesh Transformations (Blender to Unity space, etc)

## [0.2.3]
### Added
- Prefabs for models (even though the models are used in the tool, and are not supposed to be placed in a scene on their own)
- Sample scenes
  - One for all models
  - Another with an example-roadsystem, and a navigator

## [0.2.2]
### Added
- UtilTests
  - StringUtility
  - TwoDimensionalArray
  - DataCache
  - ContextDataCache

## [0.2.1]
### Added
- Link Tool button in EditorWindow

### Changed
- Link Tool MenuItem is always active
- Creating a road when an anchor is selected in the LinkTool links the road directly to the anchor, just like when an anchor gameobject is selected in the hierarchy

## [0.2.0]
### Added
- Example Assets
- Editor Window
- MenuItems
- RoadLinkTool Icon
- Wizard for selecting Road Prefab

### Fixed
- Smooth Path Generation
- Graph Generation
- Edge cases
  - Selected Link Point Deleted
  - Empty Roadsystem

## [0.1.0]
### Added
- Script files from Cybertruck Simulator

## [0.0.1]
### Added
- package.json
- Package Folders
- README
- LICENSE

[Unreleased]: https://github.com/MixusMinimax/Unity.RoadSystem/compare/v0.2.2...HEAD
[0.2.2]: https://github.com/MixusMinimax/Unity.RoadSystem/compare/v0.2.1...v0.2.2
[0.2.1]: https://github.com/MixusMinimax/Unity.RoadSystem/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/MixusMinimax/Unity.RoadSystem/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/MixusMinimax/Unity.RoadSystem/compare/v0.0.1...v0.1.0
[0.0.1]: https://github.com/MixusMinimax/Unity.RoadSystem/releases/tag/v0.0.1