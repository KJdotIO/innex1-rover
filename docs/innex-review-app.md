# Nexy Review App

Nexy is the rover review agent triggered from PR comments with `/nexy` or `/innex`. The GitHub App is named `Innex1 Reviewer`, but the review voice and status comments use Nexy.

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

The app manifest includes an inactive webhook URL because GitHub requires `hook_attributes.url` during manifest creation. Nexy does not use webhooks; GitHub Actions handles the `/nexy` and `/innex` triggers.

Set the returned app id and PEM as repository secrets, then install the app on `KJdotIO/innex1-rover`.

Use `docs/assets/innex-app-avatar.png` as the app avatar. It uses the Innova mark on a dark background so it remains visible on GitHub's light theme.

## Commands

| Command | What it does | Starts the model? |
| --- | --- | --- |
| `/nexy` or `/innex` | Reviews the PR, unless the current head SHA was already reviewed. | Sometimes |
| `/nexy please review this PR` | Same as `/nexy`; extra text is treated as review guidance. | Sometimes |
| `/nexy status` or `/innex status` | Posts the latest Nexy review state and reviewed commit. | No |
| `/nexy force review` or `/innex force review` | Runs a fresh review even if the current head SHA was already reviewed. | Yes |
| `/nexy help` or `/innex help` | Posts the command table without running a review. | No |

## Review Behaviour

One command is enough:

```text
/nexy please review this PR
```

The reviewer reads the PR title, body, diff, relevant docs, wiki pages and previous Nexy reviews. On a re-review it verifies previous findings first and avoids drip-feeding new blockers unless a newly introduced or clearly missed issue is genuinely merge-blocking.

Blocking reviews should be rare. Nexy requests changes only for high-confidence P0/P1 issues. If there are no blockers, it posts a prominent LGTM note and says the PR is mergeable from the review standpoint.

The review output is decision-first. Supporting detail such as checks, docs and unposted findings is tucked into collapsible sections so the top of the review stays easy to scan.

`/nexy status` or `/innex status` posts the latest Nexy review state without starting a model run.

If the latest Nexy review already covers the current PR head commit, a plain `/nexy` or `/innex` request is skipped and Nexy posts a short status comment instead. Use `/nexy force review` or `/innex force review` when you deliberately want another pass on the same commit.

## Model

Nexy routes through LiteLLM using the `rover-review` alias. The default backing model is:

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
