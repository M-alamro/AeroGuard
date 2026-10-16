from final_project.tracking.identity_manager import (
    IdentityManager,
)


manager = IdentityManager()


print("\n=== IDENTITY MANAGER TEST ===\n")


# --------------------------------------------------
# 1. Create first physical identity
# --------------------------------------------------

identity_id = manager.create_identity(
    byte_track_id=1,
    prediction=(100.0, 200.0),
)

print("Created:")
print(
    manager.get_identity(identity_id)
)


# --------------------------------------------------
# 2. Mark identity as LOST
# --------------------------------------------------

manager.mark_lost(
    identity_id=identity_id,
    prediction=(150.0, 220.0),
    missed=5,
)

print("\nAfter LOST:")
print(
    manager.get_identity(identity_id)
)


# --------------------------------------------------
# 3. New ByteTrack ID appears
# --------------------------------------------------

new_byte_track_id = 29

new_center = (
    175.0,
    235.0,
)

new_confidence = 0.70


# --------------------------------------------------
# 4. Search for old physical identity
# --------------------------------------------------

found_identity, distance = (
    manager.find_lost_identity(
        center=new_center,
        confidence=new_confidence,
        current_byte_track_id=new_byte_track_id,
    )
)


print("\nReassociation search:")
print(
    f"Found Identity: {found_identity}"
)

print(
    f"Distance: {distance:.2f}"
)


# --------------------------------------------------
# 5. Confirm reassociation
# --------------------------------------------------

if found_identity is not None:

    manager.confirm_reassociation(
        identity_id=found_identity,
        byte_track_id=new_byte_track_id,
    )

    manager.update(
        identity_id=found_identity,
        byte_track_id=new_byte_track_id,
        prediction=new_center,
    )


print("\nAfter Reassociation:")
print(
    manager.get_identity(identity_id)
)


print("\nCounts:")
print(
    f"Active: {manager.active_count()}"
)

print(
    f"Lost: {manager.lost_count()}"
)
