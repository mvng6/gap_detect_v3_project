Claude가 이번 작업 세션에서 변경한 코드 이력을 날짜/시간 기준으로 간단하게 정리하여 로그 문서로 저장합니다.

Steps:
1. Run `git diff --stat HEAD` to get a summary of all files changed vs last commit
2. Run `git status --short` to see any untracked new files as well
3. Run `git log --oneline -1` to get the last commit reference (baseline)
4. Run `date '+%Y-%m-%d %H:%M'` to get the current date/time
5. For each changed file, determine the type of change:
   - **신규** — new file (untracked or added)
   - **수정** — modified existing file
   - **삭제** — deleted file
6. Group changes into sections by area (launch 파일, Python 스크립트, 설정/YAML, 문서):
   - `*.launch` → **Launch 파일**
   - `*.py` → **Python 스크립트**
   - `*.yaml` / `*.yml` / `CMakeLists.txt` / `package.xml` → **설정 / 빌드**
   - `*.md` / `docs/` → **문서**
7. For each file, write ONE concise line describing what changed — no verbose explanations. Focus on the "what" not the "why". Max 1 sentence per file.
8. Save the log to `docs/CHANGELOG/WORKLOG_<YYYYMMDD>_<HHMM>.md`. The file header must include:
   - Title: `# Work Log`
   - Date: `> 날짜: <YYYY-MM-DD HH:MM>`
   - Base: `> 기준 커밋: <hash> <message>`
   Then the grouped change list.
9. Tell the user the file path.

Output format example:

```
# Work Log
> 날짜: 2026-04-02 15:30
> 기준 커밋: 816a4c5 PLAN 문서 작성

## Launch 파일
- **[신규]** `woosh_bringup/launch/woosh_navigation_system.launch` — 정식 단일 진입점, 모든 모드/설정 args 노출
- **[수정]** `woosh_navigation_mb/launch/move_base_only.launch` — 플래너 플러그인 + YAML 경로 args 추가, 하드코딩 제거
- **[수정]** `woosh_costmap/launch/global_costmap.launch` — costmap YAML 경로 args 추가
- **[수정]** `woosh_navigation_mb/launch/navigation.launch` — DEPRECATED 헤더 추가

## Python 스크립트
- **[수정]** `woosh_bringup/scripts/woosh_service_driver.py` — main() param-first 구조 전환, SLAM+nav 지원, start_base_laser_tf() 추가, start_move_base() 플래너 args 수용

## 설정 / 빌드
- **[수정]** `woosh_navigation_mb/CMakeLists.txt` — launch/config install 타겟 추가

## 문서
- **[수정]** `README.md` — 정식 실행 섹션 + 로드맵 업데이트
```

Use `$ARGUMENTS` as an optional note or label to append to the filename (e.g., `/changelog refactor` → `WORKLOG_20260402_1530_refactor.md`).
