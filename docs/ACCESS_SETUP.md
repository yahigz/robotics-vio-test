# Доступы перед запуском экспериментов

Локальная часть (OpenVINS и ORB-SLAM3 в Docker, оценка через evo) работает без доступов. Доступы нужны только для DBA-Fusion в Colab и для логирования в ClearML.

## 1. ClearML (~3 мин)
1. Зарегистрируйся на https://app.clear.ml (free tier).
2. Settings → Workspace → **Create new credentials**. Откроется блок с `api { ... }`.
3. В терминале выполни `clearml-init` и вставь этот блок. Будет создан `~/clearml.conf`. Он нужен для локального логирования и для ClearML MCP (так я смогу читать результаты экспериментов).
4. В Colab: значок ключа слева → **Secrets** → добавь пять секретов и включи для каждого «Notebook access»:
   - `CLEARML_API_ACCESS_KEY`, `CLEARML_API_SECRET_KEY` — из шага 2
   - `CLEARML_API_HOST` = `https://api.clear.ml`
   - `CLEARML_WEB_HOST` = `https://app.clear.ml`
   - `CLEARML_FILES_HOST` = `https://files.clear.ml`

## 2. GitHub-токен для Colab (~2 мин)
Репозиторий приватный (`yahigz/robotics-vio-test`), Colab клонирует его по токену. Данные и результаты через Google Drive не ходят: данные качаются прямо на диск Colab, результаты уходят в ClearML.
1. github.com → Settings → Developer settings → Personal access tokens → **Fine-grained tokens** → Generate new token.
2. Repository access: **Only select repositories** → только `robotics-vio-test`.
3. Permissions → Repository permissions → **Contents: Read-only**. Больше ничего не нужно.
4. Expiration: 30 days. Generate token и скопируй значение (показывается один раз).
5. В Colab: значок ключа слева → **Secrets** → добавь секрет `GITHUB_TOKEN` с этим значением и включи «Notebook access».
Без токена можно просто загрузить папку репозитория в `/content/vio-bench` (панель Files в Colab), ноутбук возьмёт её.

Google Drive нужен только по желанию: как кеш собранных wheels (gtsam, lietorch, droid_backends; несколько сотен МБ), чтобы не пересобирать их (~30-60 мин) в каждой новой сессии. Ноутбук сам смонтирует Drive, если `USE_DRIVE_CACHE = True`; при `False` или без Drive всё собирается с нуля.

## 3. Colab MCP (чтобы я мог запускать ноутбук сам)
1. **Перезапусти Claude Code**, чтобы подхватились новые MCP-серверы `colab-mcp` и `clearml` (проверка: `claude mcp list`).
2. Открой https://colab.research.google.com и войди в Google-аккаунт. Runtime → Change runtime type → **GPU** (T4 бесплатно; L4/A100 с Colab Pro).
3. Когда я начну запуск, colab-mcp попросит подключиться к сессии в браузере. Просто подтверди в открытой вкладке.

## 4. GitHub (для сдачи обязательно)
`gh` уже залогинен (аккаунт yahigz), репозиторий `yahigz/robotics-vio-test` приватный, `origin` настроен по ssh. Проверка: `git ls-remote origin` (входит в `pixi run check-access`).

## После этого
Скажи «запускай». Я сделаю следующее:
1. Локально прогоню OpenVINS и ORB-SLAM3 (mono-inertial: для ROVER скачана только левая камера) на 3 последовательностях ROVER, плюс sanity-check на EuRoC.
2. Запущу DBA-Fusion в Colab (код из GitHub, данные качаются на диск Colab) и заберу результаты из ClearML (`pixi run fetch-colab`).
3. Посчитаю APE/RPE и сведу всё в `results/summary/all.md` с графиками.
