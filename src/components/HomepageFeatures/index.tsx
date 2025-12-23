import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

type HomepageFeaturesProps = {
  backendUrl?: string;
  backendHealth?: {status?: string} | null;
  backendError?: string | null;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Comprehensive Curriculum',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        9 complete chapters covering Physical AI from fundamentals to advanced
        topics including ROS2, NVIDIA Isaac, and Vision-Language-Action models.
      </>
    ),
  },
  {
    title: 'AI-Powered Assistant',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Interactive RAG chatbot to help you learn. Select text and ask questions,
        get personalized explanations, and translate content to Urdu.
      </>
    ),
  },
  {
    title: 'Hands-On Learning',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Each chapter includes Python code examples, exercises, review questions,
        and real-world robotics applications you can implement.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

function BackendStatusPill({
  backendUrl,
  backendHealth,
  backendError,
}: HomepageFeaturesProps) {
  const connected = backendHealth?.status === 'OK';

  return (
    <div style={{display: 'flex', justifyContent: 'center', marginBottom: 16}}>
      <div
        style={{
          display: 'inline-flex',
          alignItems: 'center',
          gap: 8,
          padding: '8px 12px',
          borderRadius: 999,
          border: '1px solid rgba(0,0,0,0.12)',
          background: 'rgba(255,255,255,0.75)',
          backdropFilter: 'blur(6px)',
          fontSize: 13,
        }}>
        <span style={{fontWeight: 600}}>Backend:</span>
        {connected ? (
          <span>Connected ✅</span>
        ) : backendError ? (
          <span title={backendError}>Not reachable ❌</span>
        ) : (
          <span>Checking…</span>
        )}

        {backendUrl ? (
          <a
            href={`${backendUrl}/api/health`}
            target="_blank"
            rel="noreferrer"
            style={{marginLeft: 8, textDecoration: 'underline'}}>
            /api/health
          </a>
        ) : null}
      </div>
    </div>
  );
}

export default function HomepageFeatures({
  backendUrl,
  backendHealth,
  backendError,
}: HomepageFeaturesProps): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <BackendStatusPill
          backendUrl={backendUrl}
          backendHealth={backendHealth}
          backendError={backendError}
        />

        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
