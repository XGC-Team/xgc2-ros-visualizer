# HIL frozen input specimen provenance

`hil-frozen-input.json` is a sanitized specimen from the actual failed hybrid
HIL Run diagnosed by the man2 visualizer return. It retains:

- the projected eight-Robot display roster (`slotId`, `namespace`, `kind`,
  `visualization`, `hybridSource`);
- the frozen Session context Robots with their explicit profile ids and source
  parameters;
- the explicit `settings.robotProfiles` remote `state.sources.arPose`
  selections for the same Run.

Unrelated Session, camera, Agent/device and host data were removed. The
specimen is used only by the focused native tests; it is not production
configuration. The private package/deb hashes in the man2 handoff are
diagnostic provenance only and are never an input to the published product.
