"""
Auth request / response schemas.

Rules:
  - hashed_password is NEVER included in any response schema.
  - Emails are validated as a valid email address (Pydantic EmailStr) and
    stored in lowercase — normalised before hitting the DB.
  - LoginRequest does NOT distinguish "wrong password" from "no such email"
    in its response to prevent user enumeration.
"""

from pydantic import BaseModel, ConfigDict, EmailStr, field_validator
from pydantic.alias_generators import to_camel


class RegisterRequest(BaseModel):
    model_config = ConfigDict(alias_generator=to_camel, populate_by_name=True)

    email: EmailStr
    password: str

    @field_validator("email", mode="before")
    @classmethod
    def lower_email(cls, v: str) -> str:
        return v.strip().lower()


class LoginRequest(BaseModel):
    model_config = ConfigDict(alias_generator=to_camel, populate_by_name=True)

    email: EmailStr
    password: str

    @field_validator("email", mode="before")
    @classmethod
    def lower_email(cls, v: str) -> str:
        return v.strip().lower()


class UserResponse(BaseModel):
    """
    Safe public representation of a user. hashed_password is deliberately absent.
    """

    model_config = ConfigDict(
        alias_generator=to_camel,
        serialize_by_alias=True,
        populate_by_name=True,
        from_attributes=True,
    )

    id: str
    email: str
    is_active: bool
    created_at: str  # ISO 8601

    @classmethod
    def from_orm_safe(cls, user) -> "UserResponse":
        return cls(
            id=user.id,
            email=user.email,
            is_active=user.is_active,
            created_at=user.created_at.isoformat() if user.created_at else "",
        )


class TokenResponse(BaseModel):
    """JWT access token returned on successful login."""

    model_config = ConfigDict(
        alias_generator=to_camel,
        serialize_by_alias=True,
        populate_by_name=True,
    )

    access_token: str
    token_type: str = "bearer"
    expires_in_minutes: int


class LogoutResponse(BaseModel):
    message: str = "Logged out. Session cookie cleared."
