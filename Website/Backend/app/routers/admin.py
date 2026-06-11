"""
Admin router — /api/admin/*

All endpoints here require role='admin'. Authenticated users without that role
receive 403 Forbidden (not 401 — they are authenticated, just not permitted).

GET /api/admin/status — platform-wide stats for an admin dashboard:
    asset count, assessment count, user count, platform version.

Future admin endpoints (not yet built):
  - POST /api/admin/users/{id}/promote — assign role='admin' (currently DB-only)
  - GET  /api/admin/users              — list all users
  - DELETE /api/admin/assessments/{id} — force-delete any assessment
"""

from fastapi import APIRouter, Depends
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from app.core.security import require_admin
from app.database import get_db
from app.models.assessment import Assessment as AssessmentModel
from app.models.asset import Asset
from app.models.user import User

router = APIRouter(prefix="/admin", tags=["admin"])


@router.get(
    "/status",
    summary="Platform status (admin only)",
)
async def admin_status(
    db: AsyncSession = Depends(get_db),
    _: User = Depends(require_admin),
) -> dict:
    """
    Returns aggregate platform stats. Requires admin role (403 otherwise).

    Counts are live — computed at request time, not cached.
    """
    asset_count_result = await db.execute(select(func.count()).select_from(Asset))
    assessment_count_result = await db.execute(
        select(func.count()).select_from(AssessmentModel)
    )
    user_count_result = await db.execute(select(func.count()).select_from(User))

    return {
        "assetCount": asset_count_result.scalar_one(),
        "assessmentCount": assessment_count_result.scalar_one(),
        "userCount": user_count_result.scalar_one(),
        "platformVersion": "0.1.0",
    }
