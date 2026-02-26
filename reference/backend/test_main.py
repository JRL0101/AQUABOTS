from fastapi.testclient import TestClient

from .main import app

client = TestClient(app)


def test_login_and_publish():
    resp = client.post("/token", data={"username": "operator", "password": "operatorpass"})
    assert resp.status_code == 200
    token = resp.json()["access_token"]
    headers = {"Authorization": f"Bearer {token}"}
    resp2 = client.post("/publish", json={"message": "hello"}, headers=headers)
    assert resp2.status_code == 200
    assert resp2.json()["status"] == "sent"


def test_publish_forbidden():
    resp = client.post("/token", data={"username": "viewer", "password": "viewerpass"})
    token = resp.json()["access_token"]
    headers = {"Authorization": f"Bearer {token}"}
    resp2 = client.post("/publish", json={"message": "hello"}, headers=headers)
    assert resp2.status_code == 403
