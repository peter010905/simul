"""Bounded shared-memory storage for image nodes on Fast DDS 2.14."""
import os
from pathlib import Path


def image_environment(package_share):
    if any(os.environ.get(key) for key in ('FASTDDS_BUILTIN_TRANSPORTS',
            'FASTDDS_DEFAULT_PROFILES_FILE','FASTRTPS_DEFAULT_PROFILES_FILE')):
        return {}
    # Jazzy's Fast DDS 2.14 reads FASTRTPS_DEFAULT_PROFILES_FILE. The newer
    # FASTDDS_DEFAULT_PROFILES_FILE name is silently ignored by that version.
    return {'FASTRTPS_DEFAULT_PROFILES_FILE':str(Path(package_share)/'config/image_transport.xml')}
