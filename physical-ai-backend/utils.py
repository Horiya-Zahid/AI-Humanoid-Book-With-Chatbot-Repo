from pydantic import BaseModel
from typing import List

class Source(BaseModel):
    module: str
    week: str
    section: str
    url: str

# Mock data for sources
MOCK_SOURCES = [
    Source(module="Module 1", week="Week 1-5", section="The Robotic Nervous System (ROS 2)", url="/docs/module-1-robotic-nervous-system/week-1"),
    Source(module="Module 2", week="Week 6-7", section="The Digital Twin (Gazebo & Unity)", url="/docs/module-2-digital-twin/week-6"),
    Source(module="Module 3", week="Week 8-10", section="The AI-Robot Brain (NVIDIA Isaac™ Platform)", url="/docs/module-3-isaac-brain/week-8"),
    Source(module="Module 4", week="Week 11-13", section="Vision-Language-Action Capstone (VLA)", url="/docs/module-4-vla-capstone/week-11"),
]

def get_rag_response(user_query: str, selected_text: str = None) -> tuple[str, List[Source]]:
    """
    Mock function to simulate RAG response from the textbook content.
    In a real implementation, this would query the vector database.
    """
    user_query_lower = user_query.lower()

    # Determine relevant sources based on query
    relevant_sources = []
    if "ros" in user_query_lower or "robot" in user_query_lower:
        relevant_sources = [s for s in MOCK_SOURCES if "ROS" in s.section or "robot" in user_query_lower]
    elif "isaac" in user_query_lower or "nvidia" in user_query_lower:
        relevant_sources = [s for s in MOCK_SOURCES if "Isaac" in s.section]
    elif "vla" in user_query_lower or "vision-language" in user_query_lower:
        relevant_sources = [s for s in MOCK_SOURCES if "VLA" in s.section]
    elif "simulation" in user_query_lower or "gazebo" in user_query_lower or "unity" in user_query_lower:
        relevant_sources = [s for s in MOCK_SOURCES if "Digital Twin" in s.section]
    else:
        relevant_sources = MOCK_SOURCES[:2]  # Default to first two sources

    if not relevant_sources:
        relevant_sources = MOCK_SOURCES

    # Generate response based on query
    if "ros" in user_query_lower or "robot" in user_query_lower:
        response = (
            f"Based on the Physical AI & Humanoid Robotics textbook, "
            f"ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. "
            f"It provides a collection of tools, libraries, and conventions that aim to simplify "
            f"creating complex and robust robot behavior across a wide variety of robot platforms. "
            f"{f'This specifically relates to the selected text: {selected_text}' if selected_text else ''}"
        )
    elif "isaac" in user_query_lower or "nvidia" in user_query_lower:
        response = (
            f"The NVIDIA Isaac™ platform is a comprehensive solution for developing, simulating, "
            f"and deploying AI-powered robots. It combines hardware (Jetson platforms), software "
            f"frameworks, and simulation tools to accelerate robotics development. "
            f"{f'As mentioned in the selected text: {selected_text}' if selected_text else ''}"
        )
    elif "vla" in user_query_lower or "vision-language" in user_query_lower:
        response = (
            f"Vision-Language-Action (VLA) systems represent the next generation of AI-powered "
            f"robots that can perceive the world (Vision), understand and reason about it (Language), "
            f"and take appropriate actions (Action) in a unified framework. "
            f"{f'The selected text "{selected_text}" is relevant to this concept' if selected_text else ''}"
        )
    else:
        response = (
            f"I can help you with information from the Physical AI & Humanoid Robotics textbook. "
            f"This includes topics on ROS 2, Digital Twins, NVIDIA Isaac Platform, and Vision-Language-Action systems. "
            f"{f'Based on your selected text: {selected_text}' if selected_text else 'What would you like to know more about?'}"
        )

    return response, relevant_sources