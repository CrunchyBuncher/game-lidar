# Discovery recordings

Regression cases from real games: `<name>.disc` (a recording made with the overlay's **Record**,
see [../../README.md](../../README.md)) next to `<name>.toml`, the game's verified profile (e.g. a
copy of `profiles/mgs_delta.toml`). The `discovery.recordings` test and `lidar_discover check`
replay each one. They expect the profile's camera to end as the confident best candidate. Only
where the camera is read counts: buffer, layout, offsets, major and translation, not the latch,
handedness, units or up axis.

`.disc` files are git-ignored. They run to hundreds of MB (a depth grid per frame, and every
distinct buffer the sampled draws had bound), so they stay local, or go in Git LFS if they
should be shared. A `.toml` without a `.disc` is skipped.
