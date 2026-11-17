#!/usr/bin/env python3
"""
Automates creating the GitHub repository, pushing the local main branch,
and adding 'rgspkaushalya' as a collaborator using GITHUB_TOKEN from ~/.env.
"""

import json
import os
import subprocess
import sys
import urllib.error
import urllib.request

ENV_FILE = os.path.expanduser("~/.env")
REPO_NAME = "EN3563-Robotics-GP7-PickAndPlace"
COLLABORATOR = "rgspkaushalya"
REPO_DIR = "/home/chirantha-akarsha/Robotics"


def load_token():
    token = os.environ.get("GITHUB_TOKEN", "").strip()
    if token:
        return token
    if os.path.exists(ENV_FILE):
        with open(ENV_FILE, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line.startswith("GITHUB_TOKEN="):
                    return line.split("=", 1)[1].strip().strip('"').strip("'")
    return ""


def gh_api(method, path, token, payload=None):
    url = f"https://api.github.com{path}"
    data = json.dumps(payload).encode("utf-8") if payload is not None else None
    req = urllib.request.Request(url, data=data, method=method)
    req.add_header("Authorization", f"Bearer {token}")
    req.add_header("Accept", "application/vnd.github+json")
    req.add_header("X-GitHub-Api-Version", "2022-11-28")
    if data is not None:
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req) as resp:
            body = resp.read().decode("utf-8")
            return resp.status, (json.loads(body) if body else {})
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8")
        try:
            err_json = json.loads(body)
        except Exception:
            err_json = {"message": body}
        return e.code, err_json


def main():
    token = load_token()
    if not token:
        print("Error: GITHUB_TOKEN not found in ~/.env.")
        sys.exit(1)

    # 1. Get authenticated GitHub user
    status, user_info = gh_api("GET", "/user", token)
    if status != 200:
        print(f"GitHub authentication failed (HTTP {status}): {user_info.get('message')}")
        sys.exit(1)
    owner = user_info["login"]
    print(f"Authenticated as GitHub user: {owner}")

    # 2. Create repository if it does not already exist
    status, repo_info = gh_api("GET", f"/repos/{owner}/{REPO_NAME}", token)
    if status == 404:
        print(f"Creating GitHub repository '{owner}/{REPO_NAME}'...")
        status, repo_info = gh_api(
            "POST",
            "/user/repos",
            token,
            {
                "name": REPO_NAME,
                "description": "EN3563 Robotics Mini Project: YASKAWA GP7 6-DoF Robot Arm Kinematics, Trajectory Planning & Gazebo Simulation",
                "private": False,
                "auto_init": False,
            },
        )
        if status not in (200, 201):
            print(f"Failed to create repository (HTTP {status}): {repo_info.get('message')}")
            sys.exit(1)
        print(f"Created repository: https://github.com/{owner}/{REPO_NAME}")
    else:
        print(f"Repository already exists: https://github.com/{owner}/{REPO_NAME}")

    # 3. Push main branch using GIT_ASKPASS so token is never stored in .git/config
    remote_url = f"https://github.com/{owner}/{REPO_NAME}.git"
    subprocess.run(["git", "remote", "remove", "origin"], cwd=REPO_DIR, stderr=subprocess.DEVNULL)
    subprocess.run(["git", "remote", "add", "origin", remote_url], cwd=REPO_DIR, check=True)

    askpass_script = "/tmp/git_askpass_helper.sh"
    with open(askpass_script, "w", encoding="utf-8") as f:
        f.write('#!/bin/sh\necho "$GITHUB_TOKEN"\n')
    os.chmod(askpass_script, 0o700)

    env = os.environ.copy()
    env["GITHUB_TOKEN"] = token
    env["GIT_ASKPASS"] = askpass_script
    env["GIT_TERMINAL_PROMPT"] = "0"

    try:
        print(f"Pushing branch 'main' to https://github.com/{owner}/{REPO_NAME} ...")
        subprocess.run(["git", "push", "-u", "origin", "main", "--force"], cwd=REPO_DIR, env=env, check=True)
    finally:
        if os.path.exists(askpass_script):
            os.remove(askpass_script)

    # 4. Invite collaborator 'rgspkaushalya'
    print(f"Granting repository access to '{COLLABORATOR}'...")
    status, collab_info = gh_api(
        "PUT",
        f"/repos/{owner}/{REPO_NAME}/collaborators/{COLLABORATOR}",
        token,
        {"permission": "push"},
    )
    if status in (201, 204):
        print(f"Successfully granted access / sent collaborator invitation to '{COLLABORATOR}'!")
    else:
        print(f"Collaborator API response (HTTP {status}): {collab_info.get('message')}")

    print(f"\nDone! Repository URL: https://github.com/{owner}/{REPO_NAME}")


if __name__ == "__main__":
    main()
