from final_project.tracking.identity_manager import (
    IdentityManager,
)


manager = IdentityManager(
    max_missed_frames=3
)


print("\n=== IDENTITY LIFECYCLE TEST ===")


# --------------------------------------------------
# Create identity
# --------------------------------------------------

identity_id = manager.create_identity(
    byte_track_id=1,
    prediction=(100.0, 200.0),
)

print("\nCreated:")
print(
    manager.get_identity(identity_id)
)


# --------------------------------------------------
# Mark LOST
# --------------------------------------------------

manager.mark_lost(
    identity_id=identity_id,
    prediction=(110.0, 205.0),
    missed=1,
)

print("\nLost:")
print(
    manager.get_identity(identity_id)
)


# --------------------------------------------------
# Still LOST
# --------------------------------------------------

manager.mark_lost(
    identity_id=identity_id,
    prediction=(120.0, 210.0),
    missed=3,
)

expired = (
    manager.expire_old_identities()
)

print("\nAt missed=3:")
print(
    f"Expired: {expired}"
)

print(
    manager.get_identity(identity_id)
)


# --------------------------------------------------
# Exceed maximum
# --------------------------------------------------

manager.mark_lost(
    identity_id=identity_id,
    prediction=(130.0, 215.0),
    missed=4,
)

expired = (
    manager.expire_old_identities()
)

print("\nAt missed=4:")
print(
    f"Expired: {expired}"
)

print(
    manager.get_identity(identity_id)
)


# --------------------------------------------------
# Counts
# --------------------------------------------------

print("\nCounts:")

print(
    f"Active: {manager.active_count()}"
)

print(
    f"Lost: {manager.lost_count()}"
)
