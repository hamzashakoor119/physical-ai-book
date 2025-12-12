from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional
from datetime import datetime, timedelta
from passlib.context import CryptContext
from jose import JWTError, jwt
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

router = APIRouter()

# Password hashing
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# JWT configuration
SECRET_KEY = os.getenv("JWT_SECRET_KEY", "your-default-secret-key-change-in-production")
ALGORITHM = os.getenv("JWT_ALGORITHM", "HS256")
ACCESS_TOKEN_EXPIRE_MINUTES = int(os.getenv("JWT_ACCESS_TOKEN_EXPIRE_MINUTES", "30"))

class UserRegistration(BaseModel):
    email: str
    password: str
    software_experience: Optional[str] = None
    hardware_experience: Optional[str] = None
    robotics_knowledge: Optional[str] = None

class UserLogin(BaseModel):
    email: str
    password: str

class UserResponse(BaseModel):
    id: str
    email: str
    software_experience: Optional[str] = None
    hardware_experience: Optional[str] = None
    robotics_knowledge: Optional[str] = None

class Token(BaseModel):
    access_token: str
    token_type: str

def verify_password(plain_password, hashed_password):
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password):
    return pwd_context.hash(password)

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

def verify_token(token: str):
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        email: str = payload.get("sub")
        if email is None:
            raise HTTPException(status_code=401, detail="Could not validate credentials")
        return email
    except JWTError:
        raise HTTPException(status_code=401, detail="Could not validate credentials")

@router.post("/auth/register", response_model=Token)
async def register(user: UserRegistration):
    """
    Register a new user with background information
    """
    # In a real implementation, this would:
    # 1. Check if user already exists
    # 2. Hash the password
    # 3. Store user in database
    # 4. Return access token

    # For now, return a mock token
    access_token = create_access_token(data={"sub": user.email})
    return {"access_token": access_token, "token_type": "bearer"}

@router.post("/auth/login", response_model=Token)
async def login(user: UserLogin):
    """
    Authenticate user and return access token
    """
    # In a real implementation, this would:
    # 1. Verify user exists in database
    # 2. Verify password
    # 3. Return access token

    # For now, return a mock token (in real implementation, verify credentials)
    access_token = create_access_token(data={"sub": user.email})
    return {"access_token": access_token, "token_type": "bearer"}

@router.get("/auth/me", response_model=UserResponse)
async def get_current_user(token: str = Depends(lambda: None)):  # This would use a proper dependency in real implementation
    """
    Get current user's information
    """
    # In a real implementation, this would:
    # 1. Verify the token
    # 2. Fetch user from database
    # 3. Return user information

    # For now, return mock user data
    return UserResponse(
        id="mock_user_id",
        email="user@example.com",
        software_experience="intermediate",
        hardware_experience="beginner",
        robotics_knowledge="basic"
    )