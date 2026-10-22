"""Explicit units and strict JSON boundaries, separate from model mathematics."""
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field

LengthMM = Annotated[float, Field(strict=True, allow_inf_nan=False, ge=1e-6, le=1e7)]
HeadingDeg = Annotated[float, Field(strict=True, allow_inf_nan=False, gt=-89, lt=89)]


class AckermannRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    wheelbase_mm: LengthMM
    front_track_mm: LengthMM
    inside_angles_deg: Annotated[list[HeadingDeg], Field(min_length=1, max_length=721)]


class AckermannSample(BaseModel):
    inside_angle_deg: float
    outside_angle_deg: float
    left_heading_deg: float
    right_heading_deg: float
    turn_direction: Literal["left", "right", "straight"]


class AckermannResponse(BaseModel):
    model: Literal["ideal_ackermann_v1"] = "ideal_ackermann_v1"
    coordinate_system: Literal["SAE_J670_Z_DOWN"] = "SAE_J670_Z_DOWN"
    length_unit: Literal["mm"] = "mm"
    angle_unit: Literal["deg"] = "deg"
    wheelbase_mm: float
    front_track_mm: float
    samples: list[AckermannSample]
