# INX One Review App

INX One is the rover review agent triggered from PR comments with `@innex`.

The workflow still runs in GitHub Actions, but it can post reviews as a GitHub App when these repository secrets are set:

- `INNEX_APP_ID`
- `INNEX_APP_PRIVATE_KEY`

Without those secrets, reviews still work, but GitHub shows them as `github-actions[bot]`.

## Create the GitHub App

Run:

```bash
scripts/create_innex_github_app.sh
```

GitHub will open the app manifest flow. After it redirects, copy the `code=` value from the URL and run:

```bash
scripts/convert_innex_github_app_manifest.sh CODE_FROM_URL
```

The conversion output contains the private key. Do not commit it.

Set the returned app id and PEM as repository secrets, then install the app on `KJdotIO/innex1-rover`.

Use `docs/assets/innex-app-avatar.png` as the app avatar. It uses the Innova mark on a dark background so it remains visible on GitHub's light theme.

## Review Behaviour

One command is enough:

```text
@innex please review this PR
```

The reviewer reads the PR title, body, diff, relevant docs, wiki pages and previous INX One reviews. On a re-review it verifies previous findings first and avoids drip-feeding new blockers unless a newly introduced or clearly missed issue is genuinely merge-blocking.

Blocking reviews should be rare. INX One requests changes only for high-confidence P0/P1 issues. If there are no blockers, it posts an LGTM-style comment and says the PR is mergeable from the review standpoint.
