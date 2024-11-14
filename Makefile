.PHONY: test lint format proxy_redis

######################
# TESTING AND COVERAGE
######################

test:
	uv run python -m pytest tests/

######################
# LINTING AND FORMATTING
######################

lint:
	uv run python -m ruff check .
	uv run python -m ruff format . --diff
	uv run python -m mypy .

format:
	uv run python -m ruff format .
	uv run python -m ruff check --select I --fix .
