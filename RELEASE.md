# Release Process

This document explains how to create releases for HID ROS 2.

## Quick Start

### Using GitHub Web Interface (Easiest)

1. Go to https://github.com/adnan-saood/hid_ros2/releases
2. Click "Draft a new release"
3. Click "Choose a tag" → Type new tag (e.g., `v0.1.0`) → Click "Create new tag"
4. Fill in:
   - **Release title:** `v0.1.0` or `Version 0.1.0`
   - **Description:** What's new in this release
5. Click "Publish release"

**GitHub automatically provides:**
- Source code (zip)
- Source code (tar.gz)

### Using Git Tags (Automated)

The repository now has an automated release workflow. Just create and push a tag:

```bash
# Create a new version tag
git tag -a v0.1.0 -m "Release version 0.1.0"

# Push the tag to GitHub
git push origin v0.1.0
```

**The workflow automatically:**
- Creates a GitHub release
- Generates source archives (zip + tar.gz)
- Generates a changelog
- Attaches downloadable files

## Version Numbering

Follow semantic versioning: `vMAJOR.MINOR.PATCH`

- `v1.0.0` - Major release (breaking changes)
- `v0.2.0` - Minor release (new features, backwards compatible)
- `v0.1.1` - Patch release (bug fixes)

## Release Checklist

Before creating a release:

- [ ] Update version in relevant files
- [ ] Update CHANGELOG.md with new features/fixes
- [ ] Run tests: `colcon test`
- [ ] Build documentation: `cd docs && make html`
- [ ] Commit all changes
- [ ] Push to main branch
- [ ] Create and push tag (or use GitHub interface)
- [ ] Verify release on GitHub

## What Gets Packaged

The source archives include:
- All source code
- Documentation source files
- Examples
- CMakeLists.txt and package.xml files
- README, LICENSE, etc.

**Excluded from archives:**
- `build/` directory
- `install/` directory
- `log/` directory
- `.git/` directory
- Python `__pycache__`
- Generated documentation (HTML)

## Release Assets

Each release provides:

1. **Source code (zip)** - For Windows users
2. **Source code (tar.gz)** - For Linux/Mac users
3. **Auto-generated changelog** - What changed since last release

## Manual Release Notes Template

```markdown
## 🎉 What's New

- Feature 1: Description
- Feature 2: Description

## 🐛 Bug Fixes

- Fix 1: Description
- Fix 2: Description

## 📚 Documentation

- [Online Documentation](https://adnan-saood.github.io/hid_ros2/)
- [Installation Guide](https://adnan-saood.github.io/hid_ros2/installation.html)
- [Interactive HID Descriptors Guide](https://adnan-saood.github.io/hid_ros2/advanced/hid_report_descriptors.html)

## 🔧 Installation

See the [installation guide](https://adnan-saood.github.io/hid_ros2/installation.html) for detailed instructions.

Quick start:
\`\`\`bash
cd ~/ros2_ws/src
git clone https://github.com/adnan-saood/hid_ros2.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
\`\`\`
```

## Testing a Release Locally

Before creating the actual release:

```bash
# Create archive locally
git archive --format=zip --prefix=hid_ros2-v0.1.0/ HEAD -o hid_ros2-v0.1.0.zip

# Extract and test
unzip hid_ros2-v0.1.0.zip
cd hid_ros2-v0.1.0
# Test build process
```

## Deleting a Release

If you need to delete a release:

1. Go to https://github.com/adnan-saood/hid_ros2/releases
2. Click on the release
3. Click "Delete" button
4. Delete the tag: `git push --delete origin v0.1.0`
5. Delete local tag: `git tag -d v0.1.0`

## Next Steps

After creating a release:
- Announce on ROS Discourse
- Update README badges if needed
- Plan next release features
