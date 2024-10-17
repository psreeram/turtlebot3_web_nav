from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import subprocess

app = FastAPI()

# Enable CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allows all origins
    allow_credentials=True,
    allow_methods=["*"],  # Allows all methods
    allow_headers=["*"],  # Allows all headers
)

class Goal(BaseModel):
    x: float
    y: float
    theta: float

@app.post("/set_goal")
async def set_goal(goal: Goal):
    # Call ROS2 Bridge to set the goal
    subprocess.run(["ros2", "run", "turtlebot3_web_nav", "ros2_bridge", "set_goal", str(goal.x), str(goal.y), str(goal.theta)])
    return {"message": "Goal set successfully"}

@app.post("/start_navigation")
async def start_navigation():
    # Call ROS2 Bridge to start navigation
    subprocess.run(["ros2", "run", "turtlebot3_web_nav", "ros2_bridge", "start_navigation"])
    return {"message": "Navigation started"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)