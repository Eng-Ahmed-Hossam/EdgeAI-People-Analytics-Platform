from app.routers.admin import router as admin_router
from app.routers.assets import router as assets_router
from app.routers.assessments import router as assessments_router
from app.routers.auth import router as auth_router
from app.routers.valuation import router as valuation_router

__all__ = [
    "admin_router",
    "assets_router",
    "assessments_router",
    "auth_router",
    "valuation_router",
]
