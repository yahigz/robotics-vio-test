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

## 2. Google Drive через rclone (~2 мин)
Нужен, чтобы залить данные ROVER (EuRoC/ASL) на Drive для Colab.
```
rclone config
# n (new remote) → name: gdrive → storage: drive → client_id/secret: пусто
# scope: 1 (full access) → остальное по умолчанию → auto config: y → войти в браузере
rclone lsd gdrive:        # проверка
```
Дальше я сам заливаю в `gdrive:vio-bench/data/...`.

## 3. Colab MCP (чтобы я мог запускать ноутбук сам)
1. **Перезапусти Claude Code**, чтобы подхватились новые MCP-серверы `colab-mcp` и `clearml` (проверка: `claude mcp list`).
2. Открой https://colab.research.google.com и войди в Google-аккаунт. Runtime → Change runtime type → **GPU** (T4 бесплатно; L4/A100 с Colab Pro).
3. Когда я начну запуск, colab-mcp попросит подключиться к сессии в браузере. Просто подтверди в открытой вкладке.

## 4. GitHub (по желанию сейчас, для сдачи обязательно)
`gh` уже залогинен (аккаунт yahigz). Скажи, какое имя и видимость (public/private) дать репозиторию. Я создам его и запушу. Для приватного репозитория ноутбук возьмёт скрипты с Drive, токен в Colab не понадобится.

## После этого
Скажи «запускай». Я сделаю следующее:
1. Локально прогоню OpenVINS и ORB-SLAM3 (mono-inertial и stereo-inertial) на 3 последовательностях ROVER, плюс sanity-check на EuRoC.
2. Залью данные на Drive и запущу DBA-Fusion в Colab с логированием в ClearML.
3. Посчитаю APE/RPE и сведу всё в `results/summary/all.md` с графиками.
