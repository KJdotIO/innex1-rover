# Innex Review App

Innex is the rover review agent triggered from PR comments with `/innex`. Internally we may call it Nexy, but GitHub-facing comments should use Innex.

The workflow still runs in GitHub Actions, but it can post reviews as a GitHub App when these repository secrets are set:

- `INNEX_APP_ID`
- `INNEX_APP_PRIVATE_KEY`

Without those secrets, reviews still work, but GitHub shows them as `github-actions[bot]`.

## Create the GitHub App

Run:

```bash
scripts/create_innex_github_app.sh
```

GitHub will open the app manifest flow for the `KJdotIO` account. After it redirects, copy either the full redirect URL or the `code=` value from the URL and run:

```bash
scripts/convert_innex_github_app_manifest.sh 'PASTE_REDIRECT_URL_OR_CODE_HERE'
```

The conversion output contains the private key. Do not commit it.

The app manifest includes an inactive webhook URL because GitHub requires `hook_attributes.url` during manifest creation. Innex does not use webhooks; GitHub Actions handles the `/innex` trigger.

Set the returned app id and PEM as repository secrets, then install the app on `KJdotIO/innex1-rover`.

Use `docs/assets/innex-app-avatar.png` as the app avatar. It uses the Innova mark on a dark background so it remains visible on GitHub's light theme.

## Review Behaviour

One command is enough:

```text
/innex please review this PR
```

The reviewer reads the PR title, body, diff, relevant docs, wiki pages and previous Innex reviews. On a re-review it verifies previous findings first and avoids drip-feeding new blockers unless a newly introduced or clearly missed issue is genuinely merge-blocking.

Blocking reviews should be rare. Innex requests changes only for high-confidence P0/P1 issues. If there are no blockers, it posts a prominent LGTM note and says the PR is mergeable from the review standpoint.

`/innex status` posts the latest Innex review state without starting a model run.

If the latest Innex review already covers the current PR head commit, a plain `/innex` request is skipped and Innex posts a short status comment instead. Use `/innex force review` when you deliberately want another pass on the same commit.

## Model

Innex routes through LiteLLM using the `rover-review` alias. The default backing model is:

```yaml
model: openai/gpt-5.2
```

That is the normal review model. Keep heavier models for a future explicit deep-review mode if we decide the extra cost is worth it.

## Local Cleanup

After local experiments, run:

```bash
scripts/clean_innex_review_artifacts.sh
```

It removes temporary review artefacts such as local wiki checkouts, Firecrawl scratch data, generated review JSON and the one-time GitHub App conversion file.
