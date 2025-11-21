# Contributing

## Setup

See README.md for installation instructions.

## Workflow

1. Create branch: `git checkout -b feature/your-feature`
2. Make changes and test
3. Commit: `git commit -m "Add: description"`
4. Push: `git push origin feature/your-feature`
5. Create pull request on GitHub

## Commit Messages

Prefix commits with:

- `Add:` New features
- `Fix:` Bug fixes
- `Update:` Changes to existing code
- `Remove:` Deletions
- `Docs:` Documentation
- `Refactor:` Code restructuring

Examples:

- `Add: obstacle avoidance algorithm`
- `Fix: robot spawning position`
- `Update: camera parameters`

## Before Pushing

- [ ] Code builds without errors
- [ ] Test world launches successfully
- [ ] Changes tested in simulation
- [ ] Documentation updated
- [ ] Debug statements removed

## Adding Code

### Python Nodes

Place in the appropriate package subdirectory and update `setup.py`:

```python
entry_points={
    'console_scripts': [
        'your_node = package_name.your_node:main',
    ],
},
```

### Launch Files

Add to `launch/` directory and ensure it's installed in `CMakeLists.txt`:

```cmake
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
```

### World Files

Add to `lunabot_simulation/worlds/` and ensure it's installed in `CMakeLists.txt`.

## Questions

Open an issue on GitHub or contact the software lead.
