from typing import Dict
import hashlib
import base64
from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from pydantic import BaseModel

# In-memory user store for demo purposes
users_db: Dict[str, Dict[str, str]] = {
    "operator": {
        "password": hashlib.sha256("operatorpass".encode()).hexdigest(),
        "role": "operator",
    },
    "viewer": {
        "password": hashlib.sha256("viewerpass".encode()).hexdigest(),
        "role": "viewer",
    },
}

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")


class Token(BaseModel):
    access_token: str
    token_type: str = "bearer"


class User(BaseModel):
    username: str
    role: str


def verify_password(plain: str, hashed: str) -> bool:
    return hashlib.sha256(plain.encode()).hexdigest() == hashed


def authenticate_user(username: str, password: str) -> User | None:
    user = users_db.get(username)
    if not user or not verify_password(password, user["password"]):
        return None
    return User(username=username, role=user["role"])


def create_token(user: User) -> str:
    token = f"{user.username}:{user.role}".encode()
    return base64.b64encode(token).decode()


def decode_token(token: str) -> User | None:
    try:
        decoded = base64.b64decode(token.encode()).decode()
        username, role = decoded.split(":")
        return User(username=username, role=role)
    except Exception:  # pragma: no cover - invalid token
        return None


async def get_current_user(token: str = Depends(oauth2_scheme)) -> User:
    user = decode_token(token)
    if user is None:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Invalid token")
    return user


def require_role(role: str):
    def _require(user: User = Depends(get_current_user)) -> User:
        if user.role != role:
            raise HTTPException(status_code=403, detail="Not enough permissions")
        return user

    return _require
