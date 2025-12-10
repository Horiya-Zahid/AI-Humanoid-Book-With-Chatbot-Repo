import React from 'react';
import Layout from '@theme-original/Layout';
import RagChatbot from '@site/src/components/RagChatbot';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props} />
      <RagChatbot />
    </>
  );
}