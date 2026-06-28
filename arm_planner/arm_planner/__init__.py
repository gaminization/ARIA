# ARIA arm_planner — extend namespace across dist-packages (msgs) and site-packages (source)
try:
    import pkgutil
    __path__ = pkgutil.extend_path(__path__, __name__)
except Exception:
    pass
