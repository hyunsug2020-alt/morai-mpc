# Git backup before changes

Before changing project source, preserve the current project in Git using the backup service at `http://localhost:8787`.

- Source: `/home/david/morai_competition`; destination: `backups/morai_competition`; backup branch: `codex-backups`. If that branch is absent, create it from the service's default branch.
- Run the preview first. Check the selected scope and exclusions, then upload the snapshot. Confirm that the backup job completed and its remote commit was verified before reporting an upload as successful.
- Respect the service's exclusions for credentials, recordings, model weights and build outputs. Never print tokens or put credentials in commits.
- Keep the project's working-tree files, index and current branch intact. Use the service's isolated Git repository for backup commits.
- If remote upload fails, protect the complete snapshot with a persistent local Git reference before editing. Report the remote failure clearly; a local backup is not a successful GitHub upload. If neither a remote nor a verified local backup can be made, stop before changing source.
- Repeat this backup before each new batch of source edits. An identical, verified snapshot needs no duplicate commit.
