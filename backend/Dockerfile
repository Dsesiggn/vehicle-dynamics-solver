FROM python:3.12-slim
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
WORKDIR /app
COPY backend/requirements.txt backend/constraints.txt backend/
RUN pip install --no-cache-dir -r backend/requirements.txt
COPY backend/vehicle_dynamics backend/vehicle_dynamics
COPY run_hybrid.py .
COPY suspension-studio/index.html suspension-studio/steering.html suspension-studio/engineering.html suspension-studio/
COPY suspension-studio/src suspension-studio/src
COPY suspension-studio/vendor suspension-studio/vendor
COPY suspension-studio/docs/STEERING.md suspension-studio/docs/COORDINATES.md suspension-studio/docs/INTERFERENCE.md suspension-studio/docs/TIRE-TREAD.md suspension-studio/docs/
COPY suspension-studio/README.md suspension-studio/README.md
USER 10001:10001
EXPOSE 8000
CMD ["python", "run_hybrid.py", "--host", "0.0.0.0", "--port", "8000"]
