#!/bin/bash
# sync-env.sh
ENV_VALUE=$(grep PKG_NAME .vscode/.env-DEV | cut -d '=' -f2)
# 使用 jq 更新 settings.json
jq --arg val "$ENV_VALUE" '.["PKG_NAME"]=$val' .vscode/settings.json > .vscode/settings.json.tmp && mv .vscode/settings.json.tmp .vscode/settings.json
