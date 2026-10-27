# Security policy

tpt-teleop-urban controls vehicles and records public-space video. Security
reports are taken seriously, and a fault in the safety or privacy path can
affect people's physical safety or privacy.

## Reporting a vulnerability

Do not open a public issue for a security problem.

Report it privately through GitHub's private vulnerability reporting on this
repository (Security tab → "Report a vulnerability"). The repository owner must
have private vulnerability reporting enabled under Settings → Code security.

Include what you found, the steps to reproduce it, and the commit affected.
You will get an acknowledgement, and a fix will be scheduled before any public
disclosure.

## Scope

In scope: the safety guardian and actuation paths, the compliance audit ledger
and its hash chain, the privacy blur pipeline (any path that lets unblurred
faces or license plates reach disk or the teleop operator), V2X parsing of
untrusted radio input, and the teleop adapter's handling of control commands.

## Supported versions

Pre-release. Only the `master` branch receives fixes.
